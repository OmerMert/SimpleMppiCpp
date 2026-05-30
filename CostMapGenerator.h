#ifndef COSTMAP_GENERATOR_H
#define COSTMAP_GENERATOR_H

#pragma once

#include <vector>
#include <string>
#include <utility>

struct Obstacle; // Forward declaration (defined in MPPIController.h: {float x, y, r})

/**
 * @brief Loads and manages a global cost map from a CSV grid file.
 *
 * The CSV file is a 2D array of 0s and 1s (rows x cols), comma-separated.
 * Row 0 = y_min (bottom of map), last row = y_max (top).
 * Col 0 = x_min (left), last col = x_max (right).
 *
 * Obstacle circles are automatically extracted from the grid via
 * connected-component analysis. These feed directly into MPPI's
 * existing collision/CBF system (same Obstacle struct).
 */

class CostMapGenerator {
public:
    /**
     * @param filepath       Path to costmap CSV file
     * @param resolution     Meters per cell
     * @param map_x_min      World X minimum [m]
     * @param map_y_min      World Y minimum [m]
     * @param gradient_margin Soft cost zone around obstacles [m] (for RL observation)
     */
    CostMapGenerator(const std::string& filepath,
                     double resolution,
                     double map_x_min, double map_y_min,
                     double gradient_margin);

    /** Get the raw 0/1 grid as loaded from CSV */
    const std::vector<std::vector<int>>& getRawGrid() const { return raw_grid; }

    /** Get the cost map with gradient margins as flat float array (for RL) */
    const std::vector<float>& getCostMap() const { return costmap_float; }

    /** Get extracted obstacle circles (for MPPI collision/CBF) */
    const std::vector<Obstacle>& getObstacles() const { return extracted_obstacles; }

    int getRows() const { return grid_rows; }
    int getCols() const { return grid_cols; }
    double getResolution() const { return resolution; }
    double getMapXMin() const { return map_x_min; }
    double getMapYMin() const { return map_y_min; }

    /** Convert world (x,y) -> grid (row, col). Returns (-1,-1) if out of bounds. */
    std::pair<int, int> worldToGrid(double world_x, double world_y) const;

    /** Convert grid (row, col) -> world (x, y) at cell center. */
    std::pair<double, double> gridToWorld(int row, int col) const;

private:
    double resolution;
    double map_x_min, map_y_min;
    double gradient_margin;
    int grid_rows;
    int grid_cols;

    std::vector<std::vector<int>> raw_grid;     // Original 0/1 grid from CSV
    std::vector<float> costmap_float;            // With gradient, for RL observation
    std::vector<Obstacle> extracted_obstacles; // For MPPI

    void loadCSV(const std::string& filepath);
    void buildGradientCostMap();
    void extractObstacles();
};

#endif // COSTMAP_GENERATOR_H
