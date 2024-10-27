#include <vector>
#include <glm/glm.hpp>
#include <atomic>
#include <omp.h>
#include <iostream>

class PhysicsEngine {

public:


    PhysicsEngine(glm::vec2 g) {
        gravity = g;

    }

    ~PhysicsEngine() {}

    void addGameObject(float xPos, float yPos, float xAcc, float yAcc, float radius);

    void update(float dt, int sub_steps);

    void setGravity(glm::vec2 gravity) { this->gravity = gravity; }

    int getNumberOfGameObjects() { return numberOfGameObjects; }
    double getGameObjectXPosition(int index) { return gameObjectsXPositions[index]; }
    double getGameObjectYPosition(int index) { return gameObjectsYPositions[index]; }
    double getGameObjectRadius(int index) { return gameObjectsRadius[index]; }

    void clearGameObjects(){ numberOfGameObjects = 0;}
    void setThreadCount(int threadCount) { this->threadCount = threadCount; }
    int getThreadCount() { return threadCount; }
private:
    const static int maxNumberOfGameObjects = 100000;
    glm::vec2 gravity;  // Gravity force
    uint32_t numberOfGameObjects = 0;
    int threadCount = 1;
    float gameObjectsXPositions[maxNumberOfGameObjects];
    float gameObjectsYPositions[maxNumberOfGameObjects];
    float gameObjectsRadius[maxNumberOfGameObjects];
    float gameObjectsXLastPosition[maxNumberOfGameObjects];
    float gameObjectsYLastPosition[maxNumberOfGameObjects];
    float gameObjectsXAcceleration[maxNumberOfGameObjects];
    float gameObjectsYAcceleration[maxNumberOfGameObjects];


    void solveContact(uint32_t atom_1_idx, uint32_t atom_2_idx);
    bool checkIfContact(double x1, double y1, double radius1, double x2, double y2, double radius2);
    void resolveCollisionsWithWalls(int index);
    void updateObject(int index, float dt);

};


