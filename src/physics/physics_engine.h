#include <vector>
#include <glm/glm.hpp>
#include <atomic>
#include <omp.h>
#include <iostream>


class PhysicsEngine {

public:


    PhysicsEngine(glm::vec2 g) {
        gravity = g;
        for (int i = 0; i < maxCellSize * gridSize * gridSize; i++) {
            grid[i] = -1;
        }

    }

    ~PhysicsEngine() {}

    void addGameObject(float xPos, float yPos, float xAcc, float yAcc, float radius);

    void addGameObjects(std::vector<glm::vec2> gameObjects, float radius);

    void update(float dt, int sub_steps);

    glm::vec2 getGravity() { return gravity; }

    void setGravity(glm::vec2 gravity) { this->gravity = gravity; }

    void setUseGPU(bool useGPU) { this->useGPU = useGPU; }

    int getNumberOfGameObjects() { return numberOfGameObjects; }

    double getGameObjectXPosition(int index) { return gameObjectsXPositions[index]; }

    double getGameObjectYPosition(int index) { return gameObjectsYPositions[index]; }

    double getGameObjectRadius(int index) { return gameObjectsRadius[index]; }

    void setGameObjectXPosition(int index, double x) { gameObjectsXPositions[index] = x; }

    void setGameObjectYPosition(int index, double y) { gameObjectsYPositions[index] = y; }


    void clearGameObjects() { numberOfGameObjects = 0; }

    void setThreadCount(int threadCount) { this->threadCount = threadCount; }

    int getThreadCount() { return threadCount; }

    int getGridSize() { return gridSize; }

    int getGridPositionFromWorldPosition(double x, double y) const;

    int getObjectGridPosition(int index);

    int calculateAdjacentCellsPublicTest(int cell, int *adjacentCells);

    void pauseSimulation() { paused = true; }

    void resumeSimulation() { paused = false; }

    bool isSimulationPaused() { return paused; }


private:
    const static int maxNumberOfGameObjects = 1000001;
    glm::vec2 gravity;  // Gravity force
    uint32_t numberOfGameObjects = 0;
    int threadCount = 1;
    bool useGPU = false;
    const static int maxCellSize = 40;
    const static int gridSize = 120;
    float gameObjectsXPositions[maxNumberOfGameObjects];
    float gameObjectsYPositions[maxNumberOfGameObjects];
    float gameObjectsRadius[maxNumberOfGameObjects];
    float gameObjectsXLastPosition[maxNumberOfGameObjects];
    float gameObjectsYLastPosition[maxNumberOfGameObjects];
    float gameObjectsXAcceleration[maxNumberOfGameObjects];
    float gameObjectsYAcceleration[maxNumberOfGameObjects];
    int grid[maxCellSize * gridSize * gridSize];

    int objectGridPositions[maxNumberOfGameObjects];

    bool paused = false;

    void solveContact(uint32_t atom_1_idx, uint32_t atom_2_idx);

    bool checkIfContact(double x1, double y1, double radius1, double x2, double y2, double radius2);

    void resolveCollisionsWithWalls(int index);

    void updateObject(int index, float dt);

    void positionObjectsInGrid();

    void solveCollisionsBetweenTwoCells(int cell1, int cell2);

    int calculateAdjacentCells(int cell, int *adjacentCells);

    void solveCollisionsBruteForce();

    void solveCollisionsGrid();

    void calculateObjectGridPositions();

    int getObjectGridPositionFromIndex(int index);

    void cuda_solve_collisions_cpu(float *currPositionsX, float *currPositionsY, float *radii,
                                   float *lastPositionsX, float *lastPositionsY, float *accelerationX,
                                   float *accelerationY, int *grid, int sizeOfGrid, int* objectGridPositions, int maxCellSize, float xGravity,
                                   float yGravity,
                                   float dt, int substeps, int numElements);

    void cuda_update_cpu(float *xCurrPos, float *yCurrPos, float *xLastPos, float *yLastPos, float *xAcc, float *yAcc,
                         float xGravity, float yGravity, const float dt, int numElements, const float *radii);

    void cuda_calculateCellForObject_cpu(float *xPos, float *yPos, int *gridPositions, int sizeOfGrid, int maxCellSize,
                                         int numElements);

    void cuda_positionObjectsInGrid_cpu(float *xPos, float *yPos, int *grid, int *grid_positions, int sizeOfGrid,
                                        int maxCellSize, int numElements);

    void cuda_solveContactGrid_cpu(float *xPos, float *yPos, float *xPosRes, float *yPosRes, float *radii, int *grid,
                                   int sizeOfGrid, int maxCellSize, int numElements);

};