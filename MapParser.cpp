#include "MapParser.h"

#include <algorithm>
#include <assert.h>
#include <math.h>
#include <string.h>

#pragma pack(push, 1)
struct ZipCentDirEndRecord {
  uint32_t signature;
  uint16_t diskId; //    number of this disk             2 bytes
  uint16_t startDiskId; //    number of the disk with the start of the central directory  2 bytes
  uint16_t dirEntCountDisk; //    total number of entries in the central directory on this disk  2 bytes
  uint16_t dirEntCount; //    total number of entries in the central directory           2 bytes
  uint32_t dirSize; //    size of the central directory   4 bytes
  uint32_t dirOffset; //    offset of start of central directory with respect to the starting disk number        4 bytes
  uint16_t commentLength; //    .ZIP file comment length        2 bytes
};
#pragma pack(pop)
static_assert(sizeof(ZipCentDirEndRecord) == 22, "Bad central dir end size");

#pragma pack(push, 1)
struct ZipCentDirFile {
  uint32_t signature; // 0x02014b50
  uint16_t madeByVer; // version made by                 2 bytes
  uint16_t extractVer; // version needed to extract       2 bytes
  uint16_t flags; // general purpose bit flag        2 bytes
  uint16_t compressionType; // compression method              2 bytes
  uint16_t modTime; // last mod file time              2 bytes
  uint16_t modDate; // last mod file date              2 bytes
  uint32_t crc32; // crc-32                          4 bytes
  uint32_t compressedSize; // compressed size                 4 bytes
  uint32_t rawSize; // uncompressed size               4 bytes
  uint16_t nameLength; // file name length                2 bytes
  uint16_t extraLength; // extra field length              2 bytes
  uint16_t commentLength; // file comment length             2 bytes
  uint16_t diskStartNum; // disk number start               2 bytes
  uint16_t internalAttributes; // internal file attributes        2 bytes
  uint32_t externalAttributes; // external file attributes        4 bytes
  uint32_t localHeaderOffset; // relative offset of local header 4 bytes
  // variable length: file name
  // variable length: extra field
  // variable length: file comment
};
#pragma pack(pop)
static_assert(sizeof(ZipCentDirFile) == 46, "Bad central dir file size");

#pragma pack(push, 1)
struct ZipLocalFile {
  uint32_t signature; // 0x02014b50
  uint16_t extractVer; // version needed to extract       2 bytes
  uint16_t flags; // general purpose bit flag        2 bytes
  uint16_t compressionType; // compression method              2 bytes
  uint16_t modTime; // last mod file time              2 bytes
  uint16_t modDate; // last mod file date              2 bytes
  uint32_t crc32; // crc-32                          4 bytes
  uint32_t compressedSize; // compressed size                 4 bytes
  uint32_t rawSize; // uncompressed size               4 bytes
  uint16_t nameLength; // file name length                2 bytes
  uint16_t extraLength; // extra field length              2 bytes
  // variable length: file name
  // variable length: extra field
};
#pragma pack(pop)
static_assert(sizeof(ZipLocalFile) == 30, "Bad local file size");

// Custom files
#pragma pack(push, 1)
struct MapMetaV1 {
  int version;
  int mapWidth;
  int mapHeight;
};
#pragma pack(pop)
static_assert(sizeof(MapMetaV1) == 12, "Bad MapMetaV1 size");

// Custom files
enum MapWeightFlags : uint32_t { // bitmask
  MWF_NONE = 0,
  MWF_PLASTIC = (1 << 0),
};
#pragma pack(push, 1)
struct MapWeightEntryV1 {
  int x;
  int y;
  MapWeightFlags flags;
  int reserved[5];
};
#pragma pack(pop)
static_assert(sizeof(MapWeightEntryV1) == 32, "Bad MapWeightEntryV1 size");

// Ints of the distance from each cell to the nearest wall, or 0 for walls.
// This is in row-major order: the first
// row is stored, then the 2nd, and so on.
static uint8_t wallDistances[MAP_WIDTH * MAP_HEIGHT];

// Bitmask of whether each cell is in the colleciton mask.
// If so, we can't collect from that cell
static uint8_t collectionMask[MAP_WIDTH * MAP_HEIGHT / 8];

static MapWeightEntryV1 *weights = nullptr;
static int weightCount = 0;

// Assume we won't have more than 128 weights
static bool collectedWeights[128];

bool nameMatches(const ZipCentDirFile *file, const char *name) {
  if (strlen(name) != file->nameLength) {
    return false;
  }

  const char *fileName = (const char *)file + sizeof(*file);
  return strncmp(fileName, name, file->nameLength) == 0;
}

MapLoadErrno mapLoad(const uint8_t *data, int size) {
  // Reset everything, so we don't have any dangling pointers to previous data
  weights = nullptr;
  weightCount = 0;
  memset(collectedWeights, 0, sizeof(collectedWeights));

  // This is effectively a basic ZIP parser, but we only have to care about
  // the subset of ZIP that's exported by the maptool.

  // Check the central directory end record, which is in a fixed location since
  // it doesn't have a variable-length comment.
  if (size < (int)sizeof(ZipCentDirEndRecord)) {
    return MLE_CORRUPT;
  }
  const ZipCentDirEndRecord *endRec = (const ZipCentDirEndRecord *)(data + size - sizeof(ZipCentDirEndRecord));

  if (endRec->signature != 0x06054b50) {
    return MLE_CORRUPT;
  }

  bool foundHeader = false;
  bool foundWallDistances = false;
  bool foundCollectionMask = false;

  const uint8_t *nextRecord = data + endRec->dirOffset;
  const uint8_t *end = data + size;
  for (size_t i = 0; i < endRec->dirEntCount; i++) {
    const ZipCentDirFile *file = (const ZipCentDirFile *)nextRecord;
    nextRecord += sizeof(*file);
    if (nextRecord >= end) {
      return MLE_CORRUPT;
    }

    nextRecord += file->nameLength;
    nextRecord += file->extraLength;
    nextRecord += file->commentLength;
    if (nextRecord >= end) {
      return MLE_CORRUPT;
    }

    const uint8_t *fileData = data + file->localHeaderOffset;
    const ZipLocalFile *local = (const ZipLocalFile *)fileData;
    fileData += sizeof(*local);
    if (nextRecord >= end) {
      return MLE_CORRUPT;
    }
    fileData += local->nameLength;
    fileData += local->extraLength;

    if (fileData + local->rawSize > end) {
      return MLE_CORRUPT;
    }

    if (local->compressionType != 0) {
      // Compressed files aren't supported
      return MLE_CORRUPT;
    }

    // Parse each of the files:

    // Metadata v1
    if (nameMatches(file, "mapmeta.v1.bin")) {
      if (local->rawSize != sizeof(MapMetaV1)) {
        return MLE_CORRUPT;
      }

      const MapMetaV1 *meta = (const MapMetaV1 *)fileData;
      if (meta->version != 1) {
        return MLE_INCOMPATIBLE;
      }

      if (meta->mapWidth != MAP_WIDTH || meta->mapHeight != MAP_HEIGHT) {
        return MLE_DIFFERENT_SIZE;
      }

      foundHeader = true;
      continue;
    }

    // Wall distances v1
    if (nameMatches(file, "walldist.v1.dat")) {
      if (local->rawSize != sizeof(wallDistances)) {
        return MLE_MALFORMED;
      }

      memcpy(wallDistances, fileData, local->rawSize);
      foundWallDistances = true;
      continue;
    }

    // Weights v1
    if (nameMatches(file, "weights.v1.dat")) {
      if (local->rawSize % sizeof(MapWeightEntryV1)) {
        return MLE_MALFORMED;
      }

      weights = (MapWeightEntryV1 *)fileData;
      weightCount = (int)(local->rawSize / sizeof(MapWeightEntryV1));
      continue;
    }

    // Collection mask v1
    if (nameMatches(file, "collection-mask.v1.dat")) {
      if (local->rawSize != sizeof(collectionMask)) {
        return MLE_MALFORMED;
      }

      memcpy(collectionMask, fileData, local->rawSize);
      foundCollectionMask = true;
      continue;
    }

    // Unknown file type, ignore it.
  }

  if (!foundHeader || !foundWallDistances || !foundCollectionMask) {
    return MLE_MISSING_SECTION;
  }

  return MLE_SUCCESS;
}

bool mapHasObstruction(MapCell cell) {
  // out-of-bounds cells are always considered obstructed.
  if (cell >= MAP_WIDTH * MAP_HEIGHT) {
    return true;
  }

  return wallDistances[cell] == 0;
}

bool mapHasObstruction(int x, int y) {
  if (x < 0 || y < 0 || x >= MAP_WIDTH || y >= MAP_HEIGHT)
    return true;

  return mapHasObstruction(mapPackPos(x, y));
}

// ==============================
// ======== PATH FINDING ========
// ==============================

//    A few notes about A*
// (Copying Wikipedia's notation)
// The true cost of navigating from the start to a given node is g(n)
// The estimated cost of navigating from a given cell to the destination is h(n)
// The estimated cost of navigating from the start to the end via a given cell is f(n) = g(n) + h(n)

// The cost of navigating to each cell.
// Zero means the cell hasn't been computed, 1 is the starting point, and anything >1
// is a cell that's reachable but not the starting point.
// Adjacent cells have a cost of 2, diagonals have a cost of 3 - this approximates the 1 vs 1.414 ratio of diagonals.
// This is increased near walls, to provide a buffer zone. Walls are considered passable, but at a massive cost
// multiplier in case there's no usable path found for whatever reason.
// The reason zero is unreachable is that memset can be used to clear it, which
// is something we'll do very frequently.
// Note that this uses a LOT of memory: the teensy has 1MiB of RAM, this uses 128k of it!
typedef uint32_t Cost;
static Cost cellCosts[MAP_WIDTH * MAP_HEIGHT];
static constexpr Cost STRAIGHT_COST = 2;
static constexpr Cost DIAGONAL_COST = 3;

// The heap of all the yet-to-be-searched cells, ordered by their f() cost. Highest-first.
// We store the costs here, which trades off some more memory for better performance - it saves us
// having to calculate h() all the time.
// Pick a maximum set size of 1/4 the number of cells. This is enough for a zig-zag map with 2-wide
// walls and gaps. Anything with big open areas (like all real maps) will use way less than this.
struct CostIndexPair {
  Cost cost; // The f() cost.
  MapCell index;
};
static constexpr int OPEN_SET_MAX_SIZE = MAP_WIDTH * MAP_HEIGHT / 4;
static CostIndexPair openSet[OPEN_SET_MAX_SIZE];
static int openSetSize = 0;

// This is a bitmask of whether any given cell is in the open set or not.
// Since the open set is sorted by cost, we can't easily lookup if a cell is already in it or not.
static uint8_t openSetPresent[MAP_WIDTH * MAP_HEIGHT / 8];

// Make sure this gets packed properly, or we waste a lot of memory
static_assert(sizeof(CostIndexPair) == 8, "cost-index pair too big!");

// Returns true if there's an obstacle between two points
static int getLineOfSightMinWallDist(MapCell a, MapCell b);

static Cost estimateCost(MapCell cell, MapCell end) {
  int deltaX = mapExtractX(cell) - mapExtractX(end);
  int deltaY = mapExtractY(cell) - mapExtractY(end);

  if (deltaX < 0)
    deltaX *= -1;
  if (deltaY < 0)
    deltaY *= -1;

  // Calculate the shortest path if there's no obstructions
  // Force deltaX<deltaY by swapping them. They're no longer x/y
  int shortSide = deltaX;
  int longSide = deltaY;
  if (shortSide > longSide) {
    shortSide = deltaY;
    longSide = deltaX;
  }

  // Move in a diagonal as far as we can, then go straight for the rest of the way.
  int diagonalCount = shortSide;
  int straightCount = longSide - shortSide;

  return diagonalCount * DIAGONAL_COST + straightCount * STRAIGHT_COST;
}

static Cost fCost(MapCell cell, MapCell end) {
  // We need a cast since adding two Cost values converts the result to an int for some reason?
  return (Cost)(cellCosts[cell] + estimateCost(cell, end));
}

// For push_heap etc.
static bool compareCells(const CostIndexPair &first, const CostIndexPair &second) {
  // Use > instead of < to convert this max-heap to a min-heap.
  return first.cost > second.cost;
}

static void updateCell(MapCell other, MapCell end, int dx, int dy) {
  int x = mapExtractX(other) + dx;
  int y = mapExtractY(other) + dy;

  // Is this cell out-of-bounds?
  if (x < 0 || y < 0 || x >= MAP_WIDTH || y >= MAP_HEIGHT)
    return;

  MapCell cell = mapPackPos(x, y);

  if (mapHasObstruction(cell))
    return;

  // Find out the penalty from this cell's proximity to the nearest wall.
  // This must be less than about 1<<16 to guarantee we never overflow cost.
  int wallDist = wallDistances[cell];
  float costFactor = powf(1.5f, 20.f - (float)wallDist);
  if (costFactor < 1)
    costFactor = 1;

  // Figure out the cost of navigating to 'cell' from 'other'
  Cost newCost = cellCosts[other];
  if (dx != 0 && dy != 0) {
    newCost += (int)(DIAGONAL_COST * costFactor);
  } else {
    newCost += (int)(STRAIGHT_COST * costFactor);
  }

  // ... and see if that's cheaper than the current best route.
  // Note that if current=0 then we've never investigated this
  // cell before.
  Cost current = cellCosts[cell];
  if (current != 0 && current <= newCost) {
    return;
  }

  // It is cheaper, update this cell and put it into the open set so it's neighbours
  // can be updated via the main loop.
  cellCosts[cell] = newCost;

  // If this node is already in the open set, skip it.
  // This leaves it's previous cost in the heap, which doesn't feel great - I can't
  // think of any case where this would actually break things though.
  uint8_t &bitset = openSetPresent[cell / 8];
  uint8_t bitmask = 1 << (cell % 8);
  if (bitset & bitmask)
    return;
  bitset |= bitmask;

  // Insert this node into the heap
  assert(openSetSize < OPEN_SET_MAX_SIZE);
  openSet[openSetSize++] = {
      .cost = fCost(cell, end),
      .index = cell,
  };
  std::push_heap(openSet, openSet + openSetSize, compareCells);
}

int mapFindPath(MapCell start, MapCell end) {
  // Clear the existing map
  memset(cellCosts, 0, sizeof(cellCosts));

  // Mark the starting point
  cellCosts[start] = 1;

  // Initialise the open set
  // Don't bother adding start to openSetPresent, since we're about to remove it.
  openSet[0] = CostIndexPair{
      .cost = fCost(start, end),
      .index = start,
  };
  openSetSize = 1;
  memset(openSetPresent, 0, sizeof(openSetPresent));

  // Run the A* algorithm.
  while (openSetSize > 0) {
    // Find the node that we estimate will be on the cheapest path
    std::pop_heap(openSet, openSet + openSetSize, compareCells);
    CostIndexPair pair = openSet[openSetSize - 1];
    openSetSize--;

    // Remove this node from the bitmask, so it can be added to the open set again later on.
    openSetPresent[pair.index / 8] &= ~(1 << (pair.index % 8));

    // If we've found our way to the end, the shortest path can be found
    // by following the cost values backwards.
    if (pair.index == end) {
      return cellCosts[end];
    }

    // Update all of it's neighbours
    updateCell(pair.index, end, -1, -1);
    updateCell(pair.index, end, -1, 0);
    updateCell(pair.index, end, -1, 1);
    updateCell(pair.index, end, 0, -1);
    updateCell(pair.index, end, 0, 1);
    updateCell(pair.index, end, 1, -1);
    updateCell(pair.index, end, 1, 0);
    updateCell(pair.index, end, 1, 1);
  }

  // We couldn't find a path :(
  return -1;
}

int mapFindNearestWeight(MapCell start, MapCell *end) {
  // Clear the existing map
  memset(cellCosts, 0, sizeof(cellCosts));

  // Mark the starting point
  cellCosts[start] = 1;

  // Initialise the open set
  // Don't bother adding start to openSetPresent, since we're about to remove it.
  openSet[0] = CostIndexPair{
      .cost = 1,
      .index = start,
  };
  openSetSize = 1;
  memset(openSetPresent, 0, sizeof(openSetPresent));

  // Run the naive pathfinding algorithm, filling in every cell in the grid.
  // We can't use A* since we have multiple target positions.
  // Since our main search uses post-smoothing, this isn't exactly accurate - but it's good enough
  // to pick a weight, which we'll then navigate to using mapFindPath.
  while (openSetSize > 0) {
    // Grab the last-added node. This is both easy (we don't have to swap
    // items in the list) and helps with cache locality, since we're updating
    // nearby cells repeatedly.
    std::pop_heap(openSet, openSet + openSetSize, compareCells);
    CostIndexPair pair = openSet[openSetSize - 1];
    openSetSize--;

    // Remove this node from the bitmask, so it can be added to the open set again later on.
    openSetPresent[pair.index / 8] &= ~(1 << (pair.index % 8));

    // Update all of it's neighbours
    // Use the start node as the target for the weight function, so we prioritise nodes
    // near the start. This makes the map slowly expand, which should prevent anything
    // weird from happening (updating with the last node caused an overflow of the open set
    // due to the weird spiral pattern it made).
    updateCell(pair.index, start, -1, -1);
    updateCell(pair.index, start, -1, 0);
    updateCell(pair.index, start, -1, 1);
    updateCell(pair.index, start, 0, -1);
    updateCell(pair.index, start, 0, 1);
    updateCell(pair.index, start, 1, -1);
    updateCell(pair.index, start, 1, 0);
    updateCell(pair.index, start, 1, 1);
  }

  // Go through the weights, and find the one with the lowest cost
  MapCell bestWeightCell = 0;
  Cost bestWeightCost = 0;
  for (int i = 0; i < weightCount; i++) {
    const MapWeightEntryV1 *weight = &weights[i];

    // Weights should always be in-range, but check just in case.
    if (weight->x < 0 || weight->y < 0 || weight->x >= MAP_WIDTH || weight->y >= MAP_HEIGHT)
      continue;

    // Ignore plastic weights
    if (weight->flags & MWF_PLASTIC)
      continue;

    // Ignore weights we've already collected
    if (collectedWeights[i])
      continue;

    MapCell weightCell = mapPackPos(weight->x, weight->y);
    Cost cost = cellCosts[weightCell];

    // Is the weight unreachable?
    if (cost == 0)
      continue;

    // Is this the first reachable weight, or is it closer than
    // the previous closest one?
    if (bestWeightCost == 0 || cost < bestWeightCost) {
      bestWeightCost = cost;
      bestWeightCell = weightCell;
    }
  }

  // Did we find at least one reachable weight?
  if (bestWeightCost != 0) {
    *end = bestWeightCell;
    return bestWeightCost;
  }

  // We couldn't find a path :(
  return -1;
}

int mapGetCellCost(MapCell cell) {
  return cellCosts[cell];
}

MapCell mapPathNextSmooth(MapCell cell) {
  // Apply basic smoothing via line-of-sight checks

  // invariant: iter is always within a line of sight
  // Start one cell forwards, so we'll always make progress
  // even if all our subsequent line-of-sight checks fail.
  // This can notably happen with the min-wall-distance stuff.
  MapCell iter = mapPathNext(cell);

  // Are we stuck at this position? Then line-of-sight checks won't help.
  if (iter == cell) {
    return iter;
  }

  int maxWallDistance = 0;

  int count = 0;
  while (true) {
    MapCell next = mapPathNext(iter);

    // Have we found a clear path back to the starting cell?
    if (next == iter) {
      return iter;
    }

    // If the wall distance increases a bunch, stop the path. This will help us
    // stay away from walls whenever possible, even if we were near one at the
    // start of the segment.
    int wallDist = wallDistances[next];
    if (maxWallDistance < wallDist)
      maxWallDistance = wallDist;

    if (maxWallDistance < 10) {
      // Stop at any decrease, unless it forces a very short path
      if (wallDist < maxWallDistance && count >= 3) {
        return iter;
      }
    } else if (maxWallDistance < 20) {
      // Allow a small decrease
      if (wallDist < maxWallDistance - 3 && count >= 3) {
        return iter;
      }
    } else if (maxWallDistance < 30) {
      // Allow a moderate decrease
      if (wallDist < maxWallDistance - 10 && count >= 3) {
        return iter;
      }
    } else {
      // Allow a decrease to slightly below 30 units, to avoid lots of breaks
      if (wallDist < 26 && count >= 3) {
        return iter;
      }
    }

    // Find the minimum wall distance we're comfortable with
    // Tolerate getting closer to walls early on, to avoid lots of tiny
    // path segments.
    int minAcceptableDist;
    if (count <= 4) {
      // Avoid extremely short paths no matter what
      minAcceptableDist = 1;
    } else if (maxWallDistance < 30) {
      minAcceptableDist = maxWallDistance - 10;
    } else {
      minAcceptableDist = 23;
    }

    // Find the minimum distance to the nearest wall along the path between
    // the start node and the one we're considering.
    int minDistance = getLineOfSightMinWallDist(cell, next);

    // If next isn't within sight, then next is the furthest cell
    // that we do have LOS to.
    // Expressly check against zeros in case minAcceptableDist is negative.
    if (minDistance < minAcceptableDist || minDistance == 0) {
      return iter;
    }

    iter = next;

    // Stop after awhile, since the line-of-sight check has a linear time
    // complexity with the length of the path, thus we get a quadratic
    // time complexity when you include this loop.
    if (count++ > 100) {
      return iter;
    }
  }
}

MapCell mapPathNext(MapCell cell) {
  MapCell bestCell = cell;
  Cost bestCost = cellCosts[cell];

  int x = mapExtractX(cell);
  int y = mapExtractY(cell);

  for (int dx = -1; dx <= 1; dx++) {
    for (int dy = -1; dy <= 1; dy++) {
      int cx = x + dx;
      int cy = y + dy;
      if (cx < 0 || cy < 0 || cx >= MAP_WIDTH || cy >= MAP_HEIGHT)
        continue;

      MapCell thisCell = mapPackPos(cx, cy);
      if (cellCosts[thisCell] >= bestCost || cellCosts[thisCell] == 0)
        continue;

      bestCost = cellCosts[thisCell];
      bestCell = thisCell;
    }
  }

  return bestCell;
}

void markWeightAsCollected(MapCell cell) {
  for (int i = 0; i < weightCount; i++) {
    if (weights[i].x == mapExtractX(cell) && weights[i].y == mapExtractY(cell))
      collectedWeights[i] = true;
  }
}

bool mapIsCollectionMasked(int x, int y) {
  if (x < 0)
    x = 0;
  if (x >= MAP_WIDTH)
    x = MAP_WIDTH - 1;

  if (y < 0)
    y = 0;
  if (y >= MAP_HEIGHT)
    y = MAP_HEIGHT - 1;

  MapCell cell = mapPackPos(x, y);
  uint8_t bits = collectionMask[cell / 8];
  uint8_t mask = 1 << (cell % 8);
  return (bits & mask) != 0;
}

int getLineOfSightMinWallDist(MapCell a, MapCell b) {
  // From https://gist.github.com/bert/1085538 file plot_line.c

  int x0 = mapExtractX(a);
  int y0 = mapExtractY(a);
  int x1 = mapExtractX(b);
  int y1 = mapExtractY(b);

  int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
  int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
  int err = dx + dy; /* error value e_xy */

  int minDist = 256;

  for (;;) { /* loop */
    if (x0 < 0 || y0 < 0 || x0 >= MAP_WIDTH || y0 >= MAP_HEIGHT) {
      return 0; // Out-of-bounds
    }

    int wallDist = wallDistances[mapPackPos(x0, y0)];
    if (minDist > wallDist) {
      minDist = wallDist;
    }

    // Stop early if we hit a wall
    if (wallDist == 0) {
      return 0;
    }

    if (x0 == x1 && y0 == y1) {
      return minDist;
    }

    int e2 = 2 * err;
    if (e2 >= dy) {
      // e_xy+e_x > 0
      err += dy;
      x0 += sx;
    }
    if (e2 <= dx) {
      // e_xy+e_y < 0
      err += dx;
      y0 += sy;
    }
  }
}
