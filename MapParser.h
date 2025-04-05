#pragma once

#include <stdint.h>

/*
 * This module processes the map file (which is really just a ZIP file) into
 * a state that's usable by the pathfinding system.
 *
 * This file also contains said pathfinder.
 */

/**
 * The error codes returned by the map processing functions.
 */
enum MapLoadErrno {
  MLE_SUCCESS = 0,
  MLE_CORRUPT, // Invalid zip structure
  MLE_MALFORMED, // Invalid map nav data
  MLE_INCOMPATIBLE, // Mismatching version numbers
  MLE_DIFFERENT_SIZE, // Map size mismatch
  MLE_MISSING_SECTION, // O
};

/**
 * The width of the map, when it's viewed in "portrait orientation". This is where
 * the two robot bases are at the top.
 */
static constexpr int MAP_WIDTH = 128;
static constexpr int MAP_HEIGHT = 256;

// The physical size of the stage
static constexpr float STAGE_METRES_WIDTH = 2.45f;
static constexpr float STAGE_METRES_HEIGHT = 4.8f;

typedef uint16_t MapCell;
static_assert((1 << 16) > (MAP_WIDTH * MAP_HEIGHT), "uint16 can't store the mapcell coordinates");
inline constexpr MapCell mapPackPos(int x, int y) {
  return x + y * MAP_WIDTH;
}
inline constexpr int mapExtractX(MapCell cell) {
  return cell % MAP_WIDTH;
}
inline constexpr int mapExtractY(MapCell cell) {
  return cell / MAP_WIDTH;
}

/**
 * Parse a map file, returning a status number based on whether it was successful or not.
 *
 * The pointer to the data is saved, so it must remain valid for as long as any of
 * the other map-related functions are being called.
 */
MapLoadErrno mapLoad(const uint8_t *data, int size);

/**
 * Returns true if the given map cell is blocked.
 */
bool mapHasObstruction(MapCell cell);

/**
 * Returns true if the given map cell is blocked or is out of bounds.
 */
bool mapHasObstruction(int x, int y);

/**
 * Find a path between an arbitrary pair of map cells, using the A* algorithm.
 *
 * Returns the cost of the path, or -1 if no path was found.
 */
int mapFindPath(MapCell start, MapCell end);

/**
 * Find a path to the nearest (non-plastic) weight.
 *
 * End is a pointer to a MapCell which (if a path is found) is set to the position
 * of the nearest weight.
 *
 * This is slower than mapFindPath, hence it's best to only use it once, and remember
 * the position of the weight it found.
 */
int mapFindNearestWeight(MapCell start, MapCell *end);

/**
 * Get the navigation weight of a given cell. This is only meaningful after mapFindPath was called.
 */
int mapGetCellCost(MapCell cell);

/**
 * Iterate through the shortest path, as found by mapFindPath.
 *
 * This is first called with the end position, then repeatedly called on the value
 * it returns until you get back to the start cell.
 *
 * If there's not a valid path back from this cell, it returns the argument.
 */
MapCell mapPathNext(MapCell cell);

/**
 * Same as mapPathNext, but jumps between points that have open space between them.
 *
 * The robot behaves drastically better when following this path, as it's not making
 * a large number of small movements at different angles.
 */
MapCell mapPathNextSmooth(MapCell cell);

/**
 * If there is a weight in the given cell, mark it as collected so it will be ignored by mapFindNearestWeight.
 */
void markWeightAsCollected(MapCell cell);

/**
 * Check if a given cell is part of the collection mask.
 *
 * If so, weights detected in it should be ignored.
 */
bool mapIsCollectionMasked(int x, int y);
