#include <valarray>
#include <glm/vec2.hpp>
#include <cstring>
#include "physics_engine.h"

#include <random>


void PhysicsEngine::addGameObject(float xPos, float yPos, float xAcc, float yAcc, float radius) {
    if (xPos - radius < -1.0 || xPos + radius > 1.0 || yPos - radius < -1.0 || yPos + radius > 1.0) {
        throw std::runtime_error("Error: object out of bounds");
    }
    // # pragma omp parallel
    for (int i = 0; i < numberOfGameObjects; i++){
        if(checkIfContact(gameObjectsXPositions[i], gameObjectsYPositions[i],
                          gameObjectsRadius[i],xPos, yPos, radius)){

            return;
        }
    }

    gameObjectsXPositions[numberOfGameObjects] = xPos;
    gameObjectsYPositions[numberOfGameObjects] = yPos;
    gameObjectsXLastPosition[numberOfGameObjects] = xPos;
    gameObjectsYLastPosition[numberOfGameObjects] = yPos;
    gameObjectsXAcceleration[numberOfGameObjects] = xAcc;
    gameObjectsYAcceleration[numberOfGameObjects] = yAcc;
    gameObjectsRadius[numberOfGameObjects] = radius;
    numberOfGameObjects++;
}

void PhysicsEngine::addGameObjects(std::vector<glm::vec2> gameObjects, float radius) {
    //

    std::random_device rd;  // Random device to seed the generator
    std::mt19937 gen(rd()); // Mersenne Twister generator
    std::uniform_real_distribution<> dis(-100.0, 100.0); // Uniform distribution between -100 and 100

    // Generate a random double
    double random_value = dis(gen);
    for (const auto &gameObject : gameObjects) {
        // normal distrubution -100 to 100
        double random_accy = dis(gen);
        double  random_accx = dis(gen);
        addGameObject(gameObject.x, gameObject.y, random_accx, random_accy, radius);
    }
}

void PhysicsEngine::updateObject(int index, float dt) {
        gameObjectsXAcceleration[index] += gravity.x;
        gameObjectsYAcceleration[index] += gravity.y;

        double VELOCITY_DAMPING = 40.0f;
        const double last_update_move_x = gameObjectsXPositions[index] - gameObjectsXLastPosition[index];
        const double last_update_move_y = gameObjectsYPositions[index] - gameObjectsYLastPosition[index];
        const double new_position_x = gameObjectsXPositions[index] + last_update_move_x + (gameObjectsXAcceleration[index] - last_update_move_x * VELOCITY_DAMPING) * (dt * dt);
        const double new_position_y = gameObjectsYPositions[index] + last_update_move_y + (gameObjectsYAcceleration[index] - last_update_move_y * VELOCITY_DAMPING) * (dt * dt);

        gameObjectsXLastPosition[index] = gameObjectsXPositions[index];
        gameObjectsYLastPosition[index] = gameObjectsYPositions[index];
        gameObjectsXPositions[index] = new_position_x;
        gameObjectsYPositions[index] = new_position_y;
        gameObjectsXAcceleration[index] = 0.0f;
        gameObjectsYAcceleration[index] = 0.0f;
}

void PhysicsEngine::positionObjectsInGrid() {
    if (numberOfGameObjects == 0) {
        return;
    }
    for (int & i : grid) {
        i = -1;
    }
    for (int i = 0; i < numberOfGameObjects; i++) {
        const int cell = getGridPositionFromWorldPosition(gameObjectsXPositions[i], gameObjectsYPositions[i]);
        int iterations = 0;
        for (int j = 0; j < maxCellSize; j++) {
            iterations++;
            if (grid[cell * maxCellSize + j] == -1) {
                grid[cell * maxCellSize + j] = i;
                objectGridPositions[i] = cell;
                break;
            }

        }
        if (iterations == maxCellSize) {
            throw std::runtime_error("Error: too many objects in cell");
        }

    }

}

void PhysicsEngine::solveCollisionsBetweenTwoCells(int cell1, int cell2) {

    for (int i = 0; i < maxCellSize; i++) {
        if (grid[cell1 * maxCellSize + i] == -1) {
            break;
        }
        for (int j = 0; j < maxCellSize; j++) {
            if (grid[cell2 * maxCellSize + j] == -1) {
                break;
            }
            solveContact(grid[cell1 * maxCellSize + i], grid[cell2 * maxCellSize + j]);
        }
    }


}

int PhysicsEngine::calculateAdjacentCells(int cell, int* adjacentCells) {
    int x = cell % gridSize;
    int y = cell / gridSize;
    int adjacentCount = 0;
    for (int i = -1; i <= 1; i++) {
        for (int j = -1; j <= 1; j++) {
            // if (i == 0 && j == 0) {
            //     continue;
            // }
            int x_adj = x + i;
            int y_adj = y + j;
            if (x_adj >= 0 && x_adj < gridSize && y_adj >= 0 && y_adj < gridSize) {
                adjacentCells[adjacentCount] = y_adj * gridSize + x_adj;
                adjacentCount++;
            }
        }
    }
    return adjacentCount;
}

void PhysicsEngine::solveCollisionsBruteForce() {
    # pragma omp parallel for num_threads(threadCount)
    for (uint32_t i = 0; i < getNumberOfGameObjects() ; i++){
        for(uint32_t j=i+1; j< getNumberOfGameObjects(); j++)
            solveContact(i, j);
    }
}

void PhysicsEngine::solveCollisionsGrid() {
    # pragma omp parallel for num_threads(threadCount)
    for (int i = 0; i < gridSize * gridSize; i++) {
        // (int j = 0; j < gridSize * gridSize; j++) {
        int x = i % gridSize;
        int y = i / gridSize;
        for (int iAdj = -1; iAdj <= 1; iAdj++) {
            for (int jAdj = -1; jAdj <= 1; jAdj++) {

                int x_adj = x + iAdj;
                int y_adj = y + jAdj;

                if (x_adj >= 0 && x_adj < gridSize && y_adj >= 0 && y_adj < gridSize) {
                    solveCollisionsBetweenTwoCells(i, y_adj * gridSize + x_adj);
                }
            }
        }
    }
}

void PhysicsEngine::calculateObjectGridPositions() {
    for (int i = 0; i < getNumberOfGameObjects(); i++) {
        objectGridPositions[i] = -1;
    }
    for (int i = 0; i < gridSize * gridSize; i++) {
        if (grid[i] == -1) {
            continue;
        }
        for (int j = 0; j < maxCellSize; j++) {
            if (grid[i * maxCellSize + j] == -1) {
                break;
            }
            objectGridPositions[i] = grid[i * maxCellSize + j];
        }
    }
}


void PhysicsEngine::update(float dt, int sub_steps) {
    if (paused) {
        return;
    }
    if (useGPU) {
        cuda_solve_collisions(gameObjectsXPositions, gameObjectsYPositions, gameObjectsRadius,
                                     gameObjectsXLastPosition, gameObjectsYLastPosition,
                                     gameObjectsXAcceleration, gameObjectsYAcceleration, grid, objectGridPositions, gridSize, maxCellSize,
                                     gravity.x, gravity.y, dt, sub_steps,
                                     numberOfGameObjects);
        // cuda_solve_collisions_cpu(
        //         gameObjectsXPositions, gameObjectsYPositions, gameObjectsRadius,
        //         gameObjectsXLastPosition, gameObjectsYLastPosition, gameObjectsXAcceleration,
        //         gameObjectsYAcceleration, grid, gridSize, objectGridPositions, maxCellSize, gravity.x, gravity.y,
        //         dt, sub_steps, numberOfGameObjects
        //         );


    }
    else {
        float sub_dt = dt / (float)sub_steps;
        for (uint32_t j = 0; j < sub_steps; j++) {
//

            positionObjectsInGrid();
            //solveCollisionsBruteForce();
            solveCollisionsGrid();



            for (uint32_t i = 0; i <  getNumberOfGameObjects() ; i++) {
                updateObject(i, sub_dt);
                resolveCollisionsWithWalls(i);
            }

        }

// calculateObjectGridPositions();



    }
}

int PhysicsEngine::getGridPositionFromWorldPosition(const double x, const double y) const {
    const int xGrid = static_cast<int>((x + 1.0) * 0.5 * gridSize);
    const int yGrid = static_cast<int>((y + 1.0) * 0.5 * gridSize);
    return yGrid * gridSize + xGrid;
}


int PhysicsEngine::getObjectGridPosition(int index) {
     return objectGridPositions[index];
    //return getObjectGridPositionFromIndex(index);
}

int PhysicsEngine::calculateAdjacentCellsPublicTest(int cell, int *adjacentCells) {
    return calculateAdjacentCells(cell, adjacentCells);
}


void PhysicsEngine::solveContact(uint32_t atom_1_idx, uint32_t atom_2_idx) {
    if(atom_1_idx == atom_2_idx) {
        return;
    }
    constexpr float response_coef = 1.0f;
    constexpr float eps           = 0.00001f;
    double pos1_x = gameObjectsXPositions[atom_1_idx];
    double pos1_y = gameObjectsYPositions[atom_1_idx];
    double pos2_x = gameObjectsXPositions[atom_2_idx];
    double pos2_y = gameObjectsYPositions[atom_2_idx];
    double radius1 = gameObjectsRadius[atom_1_idx];
    double radius2 = gameObjectsRadius[atom_2_idx];
    const double dist2 = (pos2_x - pos1_x) * (pos2_x - pos1_x) + (pos2_y - pos1_y) * (pos2_y - pos1_y);
    if (dist2 > eps && dist2 < (radius1 + radius2) * (radius1 + radius2)) {
        const float dist          = sqrt(dist2);
        const float delta  = response_coef * 0.5f * (radius1 + radius2 - dist);
        double col_vec_x = (pos2_x - pos1_x) / dist * delta;
        double col_vec_y = (pos2_y - pos1_y) / dist * delta;
        gameObjectsXPositions[atom_1_idx] -= col_vec_x;
        gameObjectsYPositions[atom_1_idx] -= col_vec_y;
        gameObjectsXPositions[atom_2_idx] += col_vec_x;
        gameObjectsYPositions[atom_2_idx] += col_vec_y;
    }
}


bool PhysicsEngine::checkIfContact(double x1, double y1, double radius1, double x2, double y2, double radius2)
{
    const double dist2 = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);
    if (dist2 < (radius1 + radius2) * (radius1 + radius2)) {
        return true;
    }
    return false;
}


void PhysicsEngine::resolveCollisionsWithWalls(int index) {

    double pos_x = gameObjectsXPositions[index];
    double pos_y = gameObjectsYPositions[index];
    double radius = gameObjectsRadius[index];
    if (pos_x - radius < -1.0) {
        gameObjectsXPositions[index] = -1.0 + radius;  // Adjust position to be just outside the left wall
    } else if (pos_x + radius > 1.0) {
        gameObjectsXPositions[index] = 1.0 - radius;  // Adjust position to be just outside the right wall
    }

    if (pos_y - radius < -1.0) {
        gameObjectsYPositions[index] = -1.0 + radius;  // Adjust position to be just outside the bottom wall
    } else if (pos_y + radius > 1.0) {
        gameObjectsYPositions[index] = 1.0 - radius;  // Adjust position to be just outside the top wall
    }
}


void PhysicsEngine::cuda_solve_collisions_cpu(float *currPositionsX, float *currPositionsY, float *radii,
                                      float *lastPositionsX, float *lastPositionsY, float *accelerationX,
                                      float *accelerationY, int *grid,  int sizeOfGrid, int* objectGridPositions, int maxCellSize, float xGravity,  float yGravity,
                                      float dt, int substeps, int numElements){
    float *d_curr_positions_x = NULL; // input
    float *d_curr_positions_y= NULL; // input
    float *d_radii = NULL; // input
    float *d_last_positions_x = NULL; // input
    float *d_last_positions_y = NULL; // input
    float *d_acceleration_x = NULL; // input
    float *d_acceleration_y = NULL; // input
    int *d_grid = NULL;
    int *d_grid_res = NULL;
    float *d_result_x = NULL; // output
    float *d_result_y = NULL; // output
    int *d_grid_positions = NULL;
//
//
//    cudaMalloc((void **)&d_curr_positions_x, numElements * sizeof(float));
//    cudaMalloc((void **)&d_curr_positions_y, numElements * sizeof(float));
//    cudaMalloc((void **)&d_radii, numElements * sizeof(float));
//    cudaMalloc((void **)&d_result_x, numElements * sizeof(float));
//    cudaMalloc((void **)&d_result_y, numElements * sizeof(float));
//    cudaMalloc((void **)&d_last_positions_x, numElements * sizeof(float));
//    cudaMalloc((void **)&d_last_positions_y, numElements * sizeof(float));
//    cudaMalloc((void **)&d_acceleration_x, numElements * sizeof(float));
//    cudaMalloc((void **)&d_acceleration_y, numElements * sizeof(float));
//    cudaMalloc((void **)&d_grid,  sizeOfGrid * sizeOfGrid * maxCellSize * sizeof(int));
//    cudaMalloc((void **)&d_grid_res,  sizeOfGrid * sizeOfGrid * maxCellSize* sizeof(int));
//    cudaMalloc((void **)&d_grid_positions, numElements * sizeof(int));
//
//    cudaMemcpy(d_curr_positions_x, currPositionsX, numElements * sizeof(float), cudaMemcpyHostToDevice);
//    cudaMemcpy(d_curr_positions_y, currPositionsY, numElements * sizeof(float), cudaMemcpyHostToDevice);
//    cudaMemcpy(d_radii, radii, numElements * sizeof(float), cudaMemcpyHostToDevice);
//    cudaMemcpy(d_last_positions_x, lastPositionsX, numElements * sizeof(float), cudaMemcpyHostToDevice);
//    cudaMemcpy(d_last_positions_y, lastPositionsY, numElements * sizeof(float), cudaMemcpyHostToDevice);
//    cudaMemcpy(d_acceleration_x, accelerationX, numElements * sizeof(float), cudaMemcpyHostToDevice);
//    cudaMemcpy(d_acceleration_y, accelerationY, numElements * sizeof(float), cudaMemcpyHostToDevice);
//    cudaMemcpy(d_grid, grid, sizeOfGrid * sizeOfGrid * maxCellSize * sizeof(int), cudaMemcpyHostToDevice);
//
    d_curr_positions_x = (float*) malloc(numElements * sizeof(float));
    d_curr_positions_y = (float*) malloc(numElements * sizeof(float));
    d_radii = (float*) malloc(numElements * sizeof(float));
    d_result_x = (float*) malloc(numElements * sizeof(float));
    d_result_y = (float*) malloc(numElements * sizeof(float));
    d_last_positions_x = (float*) malloc(numElements * sizeof(float));
    d_last_positions_y = (float*) malloc(numElements * sizeof(float));
    d_acceleration_x = (float*) malloc(numElements * sizeof(float));
    d_acceleration_y = (float*) malloc(numElements * sizeof(float));
    d_grid = (int*) malloc(sizeOfGrid * sizeOfGrid * maxCellSize * sizeof(int));
    d_grid_res = (int*) malloc(sizeOfGrid * sizeOfGrid * maxCellSize * sizeof(int));
    d_grid_positions = (int*) malloc(numElements * sizeof(int));

    memcpy(d_curr_positions_x, currPositionsX, numElements * sizeof(float));
    memcpy(d_curr_positions_y, currPositionsY, numElements * sizeof(float));
    memcpy(d_radii, radii, numElements * sizeof(float));
    memcpy(d_last_positions_x, lastPositionsX, numElements * sizeof(float));
    memcpy(d_last_positions_y, lastPositionsY, numElements * sizeof(float));
    memcpy(d_acceleration_x, accelerationX, numElements * sizeof(float));
    memcpy(d_acceleration_y, accelerationY, numElements * sizeof(float));



//
//
//    dim3 blockSize(16, 16);
//    dim3 gridSize((numElements + blockSize.x - 1) / blockSize.x, (numElements + blockSize.y - 1) / blockSize.y);
//
//    float blockSize2 = 256;
//    int gridSize2 = (numElements + blockSize2 - 1) / blockSize2;
    float sub_dt = dt / (float)substeps;
//float sub_dt = dt / ;
//
    for (int j = 0; j < substeps; j++) {

//
////
//        cudaMemset(d_grid, -1, sizeOfGrid * sizeOfGrid * maxCellSize * sizeof(int));
//        cudaMemset(d_grid_res, -1, sizeOfGrid * sizeOfGrid * maxCellSize * sizeof(int));
        memset(d_grid, -1, sizeOfGrid * sizeOfGrid * maxCellSize * sizeof(int));
        memset(d_grid_res, -1, sizeOfGrid * sizeOfGrid * maxCellSize * sizeof(int));
//
//        calculateCellForObject<<<gridSize2, blockSize2>>>(d_curr_positions_x, d_curr_positions_y, d_grid_positions, sizeOfGrid, maxCellSize, numElements);
        cuda_calculateCellForObject_cpu(d_curr_positions_x, d_curr_positions_y, d_grid_positions, sizeOfGrid, maxCellSize, numElements);
//        positionObjectsInGrid<<<gridSize2, blockSize2>>>(d_curr_positions_x, d_curr_positions_y, d_grid, d_grid_positions, sizeOfGrid, maxCellSize, numElements);
        cuda_positionObjectsInGrid_cpu(d_curr_positions_x, d_curr_positions_y, d_grid, d_grid_positions, sizeOfGrid, maxCellSize, numElements);
//        cudaDeviceSynchronize();
//
//        cudaMemcpy(d_result_x, d_curr_positions_x, numElements * sizeof(float), cudaMemcpyDeviceToDevice);
//        cudaMemcpy(d_result_y, d_curr_positions_y, numElements * sizeof(float), cudaMemcpyDeviceToDevice);

        //memcpy(d_result_x, d_curr_positions_x, numElements * sizeof(float));
       //memcpy(d_result_y, d_curr_positions_y, numElements * sizeof(float));


//        solveContactGrid<<<gridSize2, blockSize2>>>(d_curr_positions_x, d_curr_positions_y, d_result_x, d_result_y,
//                d_radii, d_grid, sizeOfGrid, maxCellSize, numElements);
        cuda_solveContactGrid_cpu(d_curr_positions_x, d_curr_positions_y, d_result_x, d_result_y,
                                  d_radii, d_grid, sizeOfGrid, maxCellSize, numElements);
//        cudaDeviceSynchronize();
//        cudaMemcpy(d_curr_positions_x, d_result_x, numElements * sizeof(float), cudaMemcpyDeviceToDevice);
//        cudaMemcpy(d_curr_positions_y, d_result_y, numElements * sizeof(float), cudaMemcpyDeviceToDevice);

       // memcpy(d_curr_positions_x, d_result_x, numElements * sizeof(float));
       // memcpy(d_curr_positions_y, d_result_y, numElements * sizeof(float));
//
//        update<<<gridSize2, blockSize2>>>(d_curr_positions_x, d_curr_positions_y, d_last_positions_x,
//                d_last_positions_y, d_acceleration_x, d_acceleration_y,
//                xGravity, yGravity, sub_dt,
//                numElements, d_radii);
        cuda_update_cpu(d_curr_positions_x, d_curr_positions_y, d_last_positions_x,
                        d_last_positions_y, d_acceleration_x, d_acceleration_y,
                        xGravity, yGravity, sub_dt,
                        numElements, d_radii);
//
//        cudaDeviceSynchronize();
//
    }
//
//
//
//    cudaMemcpy(currPositionsX, d_curr_positions_x, numElements * sizeof(float), cudaMemcpyDeviceToHost);
//    cudaMemcpy(currPositionsY, d_curr_positions_y, numElements * sizeof(float), cudaMemcpyDeviceToHost);
//
//    cudaMemcpy(lastPositionsX, d_last_positions_x, numElements * sizeof(float), cudaMemcpyDeviceToHost);
//    cudaMemcpy(lastPositionsY, d_last_positions_y, numElements * sizeof(float), cudaMemcpyDeviceToHost);
//
//    cudaMemcpy(accelerationX, d_acceleration_x, numElements * sizeof(float), cudaMemcpyDeviceToHost);
//    cudaMemcpy(accelerationY, d_acceleration_y, numElements * sizeof(float), cudaMemcpyDeviceToHost);
//
//    cudaMemcpy(grid, d_grid, sizeOfGrid * sizeOfGrid * maxCellSize*sizeof(int), cudaMemcpyDeviceToHost);

    memcpy(currPositionsX, d_curr_positions_x, numElements * sizeof(float));
    memcpy(currPositionsY, d_curr_positions_y, numElements * sizeof(float));

    memcpy(lastPositionsX, d_last_positions_x, numElements * sizeof(float));
    memcpy(lastPositionsY, d_last_positions_y, numElements * sizeof(float));

    memcpy(accelerationX, d_acceleration_x, numElements * sizeof(float));
    memcpy(accelerationY, d_acceleration_y, numElements * sizeof(float));

    memcpy(grid, d_grid, sizeOfGrid * sizeOfGrid * maxCellSize * sizeof(int));
    memcpy(objectGridPositions, d_grid_positions, numElements * sizeof(int));

    //
//    cudaFree(d_curr_positions_x);
//    cudaFree(d_curr_positions_y);
//    cudaFree(d_radii);
//    cudaFree(d_result_x);
//    cudaFree(d_result_y);
//
//    cudaFree(d_last_positions_x);
//    cudaFree(d_last_positions_y);
//
//    cudaFree(d_acceleration_x);
//    cudaFree(d_acceleration_y);
//
//    cudaFree(d_grid);
//    cudaFree(d_grid_res);
//    cudaFree(d_grid_positions);

    free(d_curr_positions_x);
    free(d_curr_positions_y);
    free(d_radii);
    free(d_result_x);
    free(d_result_y);
    free(d_last_positions_x);
    free(d_last_positions_y);
    free(d_acceleration_x);
    free(d_acceleration_y);
    free(d_grid);
    free(d_grid_res);
    free(d_grid_positions);

}

void PhysicsEngine::cuda_update_cpu(float *xCurrPos, float *yCurrPos, float *xLastPos, float *yLastPos, float *xAcc,
                                    float *yAcc, float xGravity, float yGravity, const float dt, int numElements,
                                    const float *radii) {

//    int i = blockDim.x * blockIdx.x + threadIdx.x;

//    if (i < numElements) {
for (int i = 0; i < numElements; i++) {
        xAcc[i] += xGravity;
        yAcc[i] += yGravity;


        double VELOCITY_DAMPING = 40.0f;

        const double curr_pos_x = xCurrPos[i];
        const double curr_pos_y = yCurrPos[i];

        double last_update_move_x = curr_pos_x - xLastPos[i];
        double last_update_move_y = curr_pos_y - yLastPos[i];

        double new_position_x =
                curr_pos_x + last_update_move_x + (xAcc[i] - last_update_move_x * VELOCITY_DAMPING) * (dt * dt);
        double new_position_y =
                curr_pos_y + last_update_move_y + (yAcc[i] - last_update_move_y * VELOCITY_DAMPING) * (dt * dt);

        xLastPos[i] = curr_pos_x;
        yLastPos[i] = curr_pos_y;
        xCurrPos[i] = new_position_x;
        yCurrPos[i] = new_position_y;

        xAcc[i] = 0.0f;
        yAcc[i] = 0.0f;

        float pos_x = xCurrPos[i];
        float pos_y = yCurrPos[i];
        float radius = radii[i];
        if (pos_x - radius < -1.0) {
            xCurrPos[i] = -1.0 + radius;
        } else if (pos_x + radius > 1.0) {
            xCurrPos[i] = 1.0 - radius;
        }

        // Vertical walls
        if (pos_y - radius < -1.0) {
            yCurrPos[i] = -1.0 + radius;
        } else if (pos_y + radius > 1.0) {
            yCurrPos[i] = 1.0 - radius;
        }
    }


}

void
PhysicsEngine::cuda_calculateCellForObject_cpu(float *xPos, float *yPos, int *gridPositions, int sizeOfGrid, int maxCellSize,
                                      int numElements) {

//int i = blockDim.x * blockIdx.x + threadIdx.x;

//    if (i < numElements){
for (int i = 0; i < numElements; i++) {
        const int xGrid = floor((xPos[i] + 1.0) * 0.5 * sizeOfGrid);
        const int yGrid = floor((yPos[i] + 1.0) * 0.5 * sizeOfGrid);
        const int cell = yGrid * sizeOfGrid + xGrid;
        gridPositions[i] = cell;
    }

}

void PhysicsEngine::cuda_positionObjectsInGrid_cpu(float *xPos, float *yPos, int *d_grid, int *grid_positions, int sizeOfGrid,
                                          int maxCellSize, int numElements) {
    // int i = blockDim.x * blockIdx.x + threadIdx.x;


//    if (numElements == 0 || i >= sizeOfGrid * sizeOfGrid) {
//        return;
//    }

    for (int i = 0; i < sizeOfGrid * sizeOfGrid * maxCellSize; i++) {
        d_grid[i] = -1;
    }
    //for (int i = 0; i < sizeOfGrid * sizeOfGrid; i++) {

        for (int k = 0; k < numElements; k++) {
             int xGrid = (int)((xPos[k] + 1.0) * 0.5 * gridSize);
             int yGrid = (int)((yPos[k] + 1.0) * 0.5 * gridSize);
            int cell =  yGrid * gridSize + xGrid;
                int iterations = 0;
                for (int l = 0; l < maxCellSize; l++) {
                    iterations++;
                    if (d_grid[cell * maxCellSize + l] == -1) {
                        d_grid[cell * maxCellSize + l] += k + 1; // TODO: make this atomic operation
                        // atomicAdd(d_grid + i * maxCellSize + l, 1 + k);
                        break;
                    }
                }
                if (iterations == maxCellSize) {
                    printf("Max cell size reached in cell %d when adding object %d\n", cell, k);
                }
            }





    //}

}
void
PhysicsEngine::cuda_solveContactGrid_cpu(float *xPos, float *yPos, float *xPosRes, float *yPosRes, float *radii,
                                         int *grid_d, int sizeOfGrid, int maxCellSize, int numElements) {

//    int i = blockDim.x * blockIdx.x + threadIdx.x;
    if (numElements == 0) {
        return;
    }


//    if (i < sizeOfGrid * sizeOfGrid){
    for (int i = 0; i < sizeOfGrid * sizeOfGrid; i++) {

        int x = i % sizeOfGrid;
        int y = i / sizeOfGrid;

        for (int iAdj = -1; iAdj <= 1; iAdj++) {
            for (int jAdj = -1; jAdj <= 1; jAdj++) {

                int x_adj = x + iAdj;
                int y_adj = y + jAdj;

                if (x_adj >= 0 && x_adj < sizeOfGrid && y_adj >= 0 && y_adj < sizeOfGrid) {
                    int cell1 = i;
                    int cell2 = y_adj * sizeOfGrid + x_adj;

                    for (int iCell = 0; iCell < maxCellSize; iCell++) {
                        if (grid_d[cell1 * maxCellSize + iCell] == -1) {
                            break;
                        }
                        for (int jCell = 0; jCell < maxCellSize; jCell++) {
                            if (grid_d[cell2 * maxCellSize + jCell] == -1) {
                                break;
                            }
                            int obj1_idx = grid_d[cell1 * maxCellSize + iCell];
                            int obj2_idx = grid_d[cell2 * maxCellSize + jCell];

                            if (obj1_idx == -1 || obj2_idx == -1) {
                                printf("Error: obj1_idx or obj2_idx is -1\n");
                                break;
                            }
                            if (obj1_idx == obj2_idx) {
                                continue;
                            }
                            constexpr float response_coef = 1.0f;
                            constexpr float eps           = 0.00001f;
                            double pos1_x = xPos[obj1_idx];
                            double pos1_y = yPos[obj1_idx];
                            double pos2_x = xPos[obj2_idx];
                            double pos2_y = yPos[obj2_idx];
                            double radius1 = radii[obj1_idx];
                            double radius2 = radii[obj2_idx];

                            const double dist2 = (pos2_x - pos1_x) * (pos2_x - pos1_x) + (pos2_y - pos1_y) * (pos2_y - pos1_y);
                            if (dist2 > eps && dist2 < (radius1 + radius2) * (radius1 + radius2)) {
                                const float dist          = sqrt(dist2);
                                const float delta  = response_coef * 0.5f * (radius1 + radius2 - dist);
                                double col_vec_x = (pos2_x - pos1_x) / dist * delta;
                                double col_vec_y = (pos2_y - pos1_y) / dist * delta;

//                                atomicAdd(xPosRes + obj1_idx, -col_vec_x);
//                                atomicAdd(yPosRes + obj1_idx, -col_vec_y);
//                                atomicAdd(xPosRes + obj2_idx, +col_vec_x);
//                                atomicAdd(yPosRes + obj2_idx, +col_vec_y);

                                xPos[obj1_idx] -= col_vec_x; // TODO: make this atomic operation
                                yPos[obj1_idx] -= col_vec_y; // TODO: make this atomic operation
                                xPos[obj2_idx] += col_vec_x; // TODO: make this atomic operation
                                yPos[obj2_idx] += col_vec_y; // TODO: make this atomic operation

                            }
                        }

                    }
                }
            }
        }
    }

}

int PhysicsEngine::getObjectGridPositionFromIndex(int index) {
    for (int i = 0; i < gridSize * gridSize; i++) {
        for (int j = 0; j < maxCellSize; j++) {
            if (grid[i * maxCellSize + j] == -1) {
                break;
            }
            if (grid[i * maxCellSize + j] == index) {
                return i;
            }
        }
    }
    return -1;

}
