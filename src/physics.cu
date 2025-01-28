#include <cuda_runtime.h>
#include <stdio.h>


__global__ void update(float *xCurrPos, float *yCurrPos, float *xLastPos, float *yLastPos, float *xAcc, float *yAcc, float xGravity, float yGravity, const float dt, int numElements, const float *radii) {
    int i = blockDim.x * blockIdx.x + threadIdx.x;

    if (i < numElements) {
        xAcc[i] += xGravity;
        yAcc[i] += yGravity;


        double VELOCITY_DAMPING = 40.0f;

        const float curr_pos_x = xCurrPos[i];
        const float curr_pos_y = yCurrPos[i];

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


//    gameObjectsXAcceleration[index] += gravity.x;
//    gameObjectsYAcceleration[index] += gravity.y;
//
//    double VELOCITY_DAMPING = 40.0f;
//    const double last_update_move_x = gameObjectsXPositions[index] - gameObjectsXLastPosition[index];
//    const double last_update_move_y = gameObjectsYPositions[index] - gameObjectsYLastPosition[index];
//    const double new_position_x = gameObjectsXPositions[index] + last_update_move_x + (gameObjectsXAcceleration[index] - last_update_move_x * VELOCITY_DAMPING) * (dt * dt);
//    const double new_position_y = gameObjectsYPositions[index] + last_update_move_y + (gameObjectsYAcceleration[index] - last_update_move_y * VELOCITY_DAMPING) * (dt * dt);
//
//    gameObjectsXLastPosition[index] = gameObjectsXPositions[index];
//    gameObjectsYLastPosition[index] = gameObjectsYPositions[index];
//    gameObjectsXPositions[index] = new_position_x;
//    gameObjectsYPositions[index] = new_position_y;
//    gameObjectsXAcceleration[index] = 0.0f;
//    gameObjectsYAcceleration[index] = 0.0f;

}


__global__ void calculateCellForObject(float *xPos, float *yPos, int  *gridPositions, int sizeOfGrid, int maxCellSize, int numElements) {
    int i = blockDim.x * blockIdx.x + threadIdx.x;

      if (i < numElements){
        const int xGrid = floor((xPos[i] + 1.0) * 0.5 * sizeOfGrid);
        const int yGrid = floor((yPos[i] + 1.0) * 0.5 * sizeOfGrid);
        const int cell = yGrid * sizeOfGrid + xGrid;
        gridPositions[i] = cell;
      }
  }



__global__ void positionObjectsInGrid(float *xPos, float *yPos, int *grid, int *grid_positions, int sizeOfGrid, int maxCellSize, int numElements) {

    int i = blockDim.x * blockIdx.x + threadIdx.x;




    if (i >= numElements) {
        return;
    }

    // for (int k = 0; k < numElements; k++){
        // if (grid_positions[k] == i){

            const int xGrid = floor((xPos[i] + 1.0) * 0.5 * sizeOfGrid);
            const int yGrid = floor((yPos[i] + 1.0) * 0.5 * sizeOfGrid);
            const int cell = yGrid * sizeOfGrid + xGrid;
            int iterations = 0;
            for (int l = 0; l < maxCellSize; l++) {
                iterations++;
                // if (grid[cell * maxCellSize + l] == -1) {
                //     grid[cell * maxCellSize + l] = i;
                //     break;
                // }
                if (atomicCAS(grid +cell * maxCellSize + l, -1, i) == -1) {
                    break;
                }
            }
            if (iterations == maxCellSize) {
                printf("Max cell size reached in cell %d when adding object %d\n", cell, i);
            }
        // }


    // }





}

__global__ void solveContactGrid(float *xPos, float *yPos, float *xPosRes, float *yPosRes, float *radii, int *grid, int sizeOfGrid, int maxCellSize, int numElements) {
    int i = blockDim.x * blockIdx.x + threadIdx.x;
    if (numElements == 0) {
        return;
    }
    if (i < sizeOfGrid * sizeOfGrid){

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
                        if (grid[cell1 * maxCellSize + iCell] == -1) {
                            break;
                        }
                        for (int jCell = 0; jCell < maxCellSize; jCell++) {
                            if (grid[cell2 * maxCellSize + jCell] == -1) {
                                break;
                            }
                            int obj1_idx = grid[cell1 * maxCellSize + iCell];
                            int obj2_idx = grid[cell2 * maxCellSize + jCell];
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

                            const float dist2 = (pos2_x - pos1_x) * (pos2_x - pos1_x) + (pos2_y - pos1_y) * (pos2_y - pos1_y);
                            if (dist2 > eps && dist2 < (radius1 + radius2) * (radius1 + radius2)) {
                                const float dist          = sqrt(dist2);
                                const float delta  = response_coef * 0.5f * (radius1 + radius2 - dist);
                                double col_vec_x = (pos2_x - pos1_x) / dist * delta;
                                double col_vec_y = (pos2_y - pos1_y) / dist * delta;

                                atomicAdd(xPos + obj1_idx, -col_vec_x);
                                atomicAdd(yPos + obj1_idx, -col_vec_y);
                                atomicAdd(xPos + obj2_idx, +col_vec_x);
                                atomicAdd(yPos + obj2_idx, +col_vec_y);
                            }
                       }

                    }
                }
            }
        }
    }


}


extern "C" void cuda_solve_collisions(float *currPositionsX, float *currPositionsY, float *radii,
                                      float *lastPositionsX, float *lastPositionsY, float *accelerationX,
                                      float *accelerationY, int *grid,  int * gridPositions, int sizeOfGrid, int maxCellSize, float xGravity,  float yGravity,
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
    int *lock = NULL;


    cudaMalloc((void **)&d_curr_positions_x, numElements * sizeof(float));
    cudaMalloc((void **)&d_curr_positions_y, numElements * sizeof(float));
    cudaMalloc((void **)&d_radii, numElements * sizeof(float));
    cudaMalloc((void **)&d_result_x, numElements * sizeof(float));
    cudaMalloc((void **)&d_result_y, numElements * sizeof(float));
    cudaMalloc((void **)&d_last_positions_x, numElements * sizeof(float));
    cudaMalloc((void **)&d_last_positions_y, numElements * sizeof(float));
    cudaMalloc((void **)&d_acceleration_x, numElements * sizeof(float));
    cudaMalloc((void **)&d_acceleration_y, numElements * sizeof(float));
    cudaMalloc((void **)&d_grid,  sizeOfGrid * sizeOfGrid * maxCellSize * sizeof(int));
    cudaMalloc((void **)&d_grid_res,  sizeOfGrid * sizeOfGrid * maxCellSize* sizeof(int));
    cudaMalloc((void **)&d_grid_positions, numElements * sizeof(int));

    cudaMalloc((void **)&lock, sizeOfGrid * sizeOfGrid * maxCellSize * sizeof(int));

    cudaMemcpy(d_curr_positions_x, currPositionsX, numElements * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_curr_positions_y, currPositionsY, numElements * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_radii, radii, numElements * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_last_positions_x, lastPositionsX, numElements * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_last_positions_y, lastPositionsY, numElements * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_acceleration_x, accelerationX, numElements * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_acceleration_y, accelerationY, numElements * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_grid, grid, sizeOfGrid * sizeOfGrid * maxCellSize * sizeof(int), cudaMemcpyHostToDevice);





    int blockSize = 256;
    int gridSizeObjects = (numElements + blockSize - 1) / blockSize;
    int gridSizeGrid = (sizeOfGrid * sizeOfGrid + blockSize - 1) / blockSize;
    float sub_dt = dt / (float)substeps;

    for (int j = 0; j < substeps; j++) {

//
        cudaMemset(d_grid, -1, sizeOfGrid * sizeOfGrid * maxCellSize * sizeof(int));
        cudaMemset(d_grid_res, -1, sizeOfGrid * sizeOfGrid * maxCellSize * sizeof(int));

        calculateCellForObject<<<gridSizeObjects, blockSize>>>(d_curr_positions_x, d_curr_positions_y, d_grid_positions, sizeOfGrid, maxCellSize, numElements);
        //
        positionObjectsInGrid<<<gridSizeObjects, blockSize>>>(d_curr_positions_x, d_curr_positions_y, d_grid, d_grid_positions, sizeOfGrid, maxCellSize, numElements);
        cudaDeviceSynchronize();

        solveContactGrid<<<gridSizeGrid, blockSize>>>(d_curr_positions_x, d_curr_positions_y, d_result_x, d_result_y,
                                              d_radii, d_grid, sizeOfGrid, maxCellSize, numElements);
        cudaDeviceSynchronize();
        update<<<gridSizeObjects, blockSize>>>(d_curr_positions_x, d_curr_positions_y, d_last_positions_x,
                                   d_last_positions_y, d_acceleration_x, d_acceleration_y,
                                   xGravity, yGravity, sub_dt,
                                   numElements, d_radii);

        cudaDeviceSynchronize();

    }

    // print entire d_grid
    // for (int i = 0; i < sizeOfGrid * sizeOfGrid; i++) {
    //     if (grid[i * maxCellSize] == -1) {
    //         continue;
    //     }
    //     printf("Cell %d: ", i);
    //     for (int j = 0; j < maxCellSize; j++) {
    //         if (grid[i * maxCellSize + j] == -1) {
    //             break;
    //         }
    //         printf("%d ", grid[i * maxCellSize + j]);
    //     }
    //     printf("\n");
    // }



    cudaMemcpy(currPositionsX, d_curr_positions_x, numElements * sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(currPositionsY, d_curr_positions_y, numElements * sizeof(float), cudaMemcpyDeviceToHost);

    cudaMemcpy(lastPositionsX, d_last_positions_x, numElements * sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(lastPositionsY, d_last_positions_y, numElements * sizeof(float), cudaMemcpyDeviceToHost);

    cudaMemcpy(accelerationX, d_acceleration_x, numElements * sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(accelerationY, d_acceleration_y, numElements * sizeof(float), cudaMemcpyDeviceToHost);

    cudaMemcpy(grid, d_grid, sizeOfGrid * sizeOfGrid * maxCellSize*sizeof(int), cudaMemcpyDeviceToHost);
    cudaMemcpy(gridPositions, d_grid_positions, numElements * sizeof(int), cudaMemcpyDeviceToHost);

    cudaFree(d_curr_positions_x);
    cudaFree(d_curr_positions_y);
    cudaFree(d_radii);
    cudaFree(d_result_x);
    cudaFree(d_result_y);

    cudaFree(d_last_positions_x);
    cudaFree(d_last_positions_y);

    cudaFree(d_acceleration_x);
    cudaFree(d_acceleration_y);

    cudaFree(d_grid);
    cudaFree(d_grid_res);
    cudaFree(d_grid_positions);
}
