#include "CostMapGenerator.h"
#include "MPPIController.h" // for Obstacle struct
#include <fstream>
#include <sstream>
#include <iostream>
#include <cmath>
#include <queue>
#include <algorithm>
#include <stdexcept>

CostMapGenerator::CostMapGenerator(
    const std::string& filepath,
    double resolution,
    double map_x_min, double map_y_min,
    double gradient_margin)
    : resolution(resolution),
      map_x_min(map_x_min), map_y_min(map_y_min),
      gradient_margin(gradient_margin)
{
    loadCSV(filepath);
    buildGradientCostMap();
    extractObstacles();
}

void CostMapGenerator::loadCSV(const std::string& filepath) {
    std::ifstream file(filepath);
    if (!file.is_open()) {
        throw std::runtime_error("[CostMap] Cannot open: " + filepath);
    }

    std::string line;
    while (std::getline(file, line)) {
        // Remove trailing \r if present (Windows line endings)
        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }
        if (line.empty()) continue;

        std::vector<int> row;
        std::stringstream ss(line);
        std::string cell;
        while (std::getline(ss, cell, ',')) {
            row.push_back(std::stoi(cell));
        }
        raw_grid.push_back(row);
    }
    file.close();

    if (raw_grid.empty()) {
        throw std::runtime_error("[CostMap] Empty CSV file: " + filepath);
    }

    grid_rows = static_cast<int>(raw_grid.size());
    grid_cols = static_cast<int>(raw_grid[0].size());

    std::cout << "[CostMap] Loaded " << filepath
              << " -> " << grid_rows << " x " << grid_cols << " grid" << std::endl;
}

void CostMapGenerator::buildGradientCostMap() {
    costmap_float.resize(grid_rows * grid_cols, 0.0f);

    // First pass: copy raw obstacles
    for (int r = 0; r < grid_rows; ++r) {
        for (int c = 0; c < grid_cols; ++c) {
            if (raw_grid[r][c] == 1) {
                costmap_float[r * grid_cols + c] = 1.0f;
            }
        }
    }

    // Second pass: add gradient around obstacles
    if (gradient_margin <= 0.0) return;

    int margin_cells = static_cast<int>(std::ceil(gradient_margin / resolution));

    for (int r = 0; r < grid_rows; ++r) {
        for (int c = 0; c < grid_cols; ++c) {
            if (raw_grid[r][c] == 1) continue; // Already obstacle

            // Find minimum distance to any obstacle cell
            float min_dist = 1e10f;
            int r_start = std::max(0, r - margin_cells);
            int r_end   = std::min(grid_rows - 1, r + margin_cells);
            int c_start = std::max(0, c - margin_cells);
            int c_end   = std::min(grid_cols - 1, c + margin_cells);

            for (int rr = r_start; rr <= r_end; ++rr) {
                for (int cc = c_start; cc <= c_end; ++cc) {
                    if (raw_grid[rr][cc] == 1) {
                        float dx = (c - cc) * (float)resolution;
                        float dy = (r - rr) * (float)resolution;
                        float dist = std::sqrt(dx * dx + dy * dy);
                        min_dist = std::min(min_dist, dist);
                    }
                }
            }

            if (min_dist < gradient_margin) {
                float cost = 1.0f - (min_dist / (float)gradient_margin);
                costmap_float[r * grid_cols + c] = std::max(
                    costmap_float[r * grid_cols + c], cost);
            }
        }
    }
}

void CostMapGenerator::extractObstacles() {
    // Flood-fill to find connected components of obstacle cells,
    // then compute bounding circle for each component.

    std::vector<std::vector<bool>> visited(grid_rows, std::vector<bool>(grid_cols, false));

    int dx[] = {0, 0, 1, -1, 1, -1, 1, -1};
    int dy[] = {1, -1, 0, 0, 1, -1, -1, 1};

    for (int r = 0; r < grid_rows; ++r) {
        for (int c = 0; c < grid_cols; ++c) {
            if (raw_grid[r][c] != 1 || visited[r][c]) continue;

            // BFS flood fill
            std::queue<std::pair<int, int>> q;
            std::vector<std::pair<int, int>> component;
            q.push({r, c});
            visited[r][c] = true;

            while (!q.empty()) {
                auto [cr, cc] = q.front();
                q.pop();
                component.push_back({cr, cc});

                for (int d = 0; d < 8; ++d) {
                    int nr = cr + dy[d];
                    int nc = cc + dx[d];
                    if (nr >= 0 && nr < grid_rows && nc >= 0 && nc < grid_cols
                        && !visited[nr][nc] && raw_grid[nr][nc] == 1) {
                        visited[nr][nc] = true;
                        q.push({nr, nc});
                    }
                }
            }

            // Compute centroid in world coordinates
            double sum_x = 0.0, sum_y = 0.0;
            for (auto [pr, pc] : component) {
                auto [wx, wy] = gridToWorld(pr, pc);
                sum_x += wx;
                sum_y += wy;
            }
            double cx = sum_x / component.size();
            double cy = sum_y / component.size();

            // Compute radius: max distance from centroid to any cell center
            double max_dist = 0.0;
            for (auto [pr, pc] : component) {
                auto [wx, wy] = gridToWorld(pr, pc);
                double dist = std::sqrt((wx - cx) * (wx - cx) + (wy - cy) * (wy - cy));
                max_dist = std::max(max_dist, dist);
            }

            // Add half-cell diagonal to cover cell extent
            double cell_diag = resolution * std::sqrt(2.0) * 0.5;

            Obstacle obs;
            obs.x = static_cast<float>(cx);
            obs.y = static_cast<float>(cy);
            obs.r = static_cast<float>(max_dist + cell_diag);
            extracted_obstacles.push_back(obs);

            std::cout << "[CostMap] Obstacle found: center=("
                      << obs.x << ", " << obs.y << ") radius=" << obs.r
                      << " cells=" << component.size() << std::endl;
        }
    }

    std::cout << "[CostMap] Total obstacles extracted: "
              << extracted_obstacles.size() << std::endl;
}

std::pair<int, int> CostMapGenerator::worldToGrid(double world_x, double world_y) const {
    int col = static_cast<int>((world_x - map_x_min) / resolution);
    int row = static_cast<int>((world_y - map_y_min) / resolution);
    if (row < 0 || row >= grid_rows || col < 0 || col >= grid_cols) {
        return {-1, -1};
    }
    return {row, col};
}

std::pair<double, double> CostMapGenerator::gridToWorld(int row, int col) const {
    double world_x = map_x_min + (col + 0.5) * resolution;
    double world_y = map_y_min + (row + 0.5) * resolution;
    return {world_x, world_y};
}
