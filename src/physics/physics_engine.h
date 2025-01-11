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

    void update(float dt, int sub_steps);

    glm::vec2 getGravity() { return gravity; }
    void setGravity(glm::vec2 gravity) { this->gravity = gravity; }

    int getNumberOfGameObjects() { return numberOfGameObjects; }
    double getGameObjectXPosition(int index) { return gameObjectsXPositions[index]; }
    double getGameObjectYPosition(int index) { return gameObjectsYPositions[index]; }
    double getGameObjectRadius(int index) { return gameObjectsRadius[index]; }

    void setGameObjectXPosition(int index, double x) { gameObjectsXPositions[index] = x; }
    void setGameObjectYPosition(int index, double y) { gameObjectsYPositions[index] = y; }


    void clearGameObjects(){ numberOfGameObjects = 0;}
    void setThreadCount(int threadCount) { this->threadCount = threadCount; }
    int getThreadCount() { return threadCount; }
    int getGridSize() { return gridSize; }
    int getGridPositionFromWorldPosition(double x, double y) const;
    int getObjectGridPosition(int index);

    int calculateAdjacentCellsPublicTest(int cell, int *adjacentCells);


private:
    const static int maxNumberOfGameObjects = 50000;
    glm::vec2 gravity;  // Gravity force
    uint32_t numberOfGameObjects = 0;
    int threadCount = 1;
    const static int maxCellSize = 40;
    const static int gridSize = 200;
    float gameObjectsXPositions[maxNumberOfGameObjects];
    float gameObjectsYPositions[maxNumberOfGameObjects];
    float gameObjectsRadius[maxNumberOfGameObjects];
    float gameObjectsXLastPosition[maxNumberOfGameObjects];
    float gameObjectsYLastPosition[maxNumberOfGameObjects];
    float gameObjectsXAcceleration[maxNumberOfGameObjects];
    float gameObjectsYAcceleration[maxNumberOfGameObjects];
    int grid[maxCellSize * gridSize * gridSize];

    void solveContact(uint32_t atom_1_idx, uint32_t atom_2_idx);
    bool checkIfContact(double x1, double y1, double radius1, double x2, double y2, double radius2);
    void resolveCollisionsWithWalls(int index);
    void updateObject(int index, float dt);
    void positionObjectsInGrid();
    void solveCollisionsBetweenTwoCells(int cell1, int cell2);
    int calculateAdjacentCells(int cell, int *adjacentCells);

    void solveCollisionsBruteForce();
    void solveCollisionsGrid();

};


