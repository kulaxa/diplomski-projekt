#include <GL/glut.h>
#include <cmath>
#include "imgui.h"
#include "backends/imgui_impl_opengl2.h"
#include "backends/imgui_impl_glut.h"
#include "imgui_internal.h"
#include <glm/glm.hpp>
#include <fstream>
#include <thread>
#include <atomic>
#include "physics/physics_engine.h"


GLuint window;
GLuint width = 1000, height = 1000;

std::chrono::duration<double> physicsTime = std::chrono::milliseconds(16);

PhysicsEngine physicsEngine(glm::vec2(0.0, -0.1));

std::ofstream outputFile;

void MainLoopStep();



void drawGrid(int grid_size){
    glBegin(GL_LINES);
    glColor3f(0.0f, 0.0f, 0.0f);

    for (int i = 0; i <= grid_size; ++i) {
        float y = 2.0 * i / grid_size - 1.0;
        glVertex2f(-1.0, y);
        glVertex2f(1.0, y);
        glVertex2f(y, -1.0);
        glVertex2f( y,1.0);

    }

    glEnd();
}



void drawCircle(double xPos, double yPos, double radius, int num_segments, glm::vec3 color) {


    glBegin(GL_TRIANGLE_FAN );
    for (int i = 0; i < num_segments; i++) {
        float theta = 2.0f * 3.1415926f * float(i) / float(num_segments);
        float x = radius * cosf(theta);
        float y = radius * sinf(theta);
        glColor3f(color.x, color.y, color.z);
        glVertex2f(x + xPos, y + yPos);
    }
    glEnd();
}

// Display callback function
void display() {


//    drawGrid({physicsEngine.getGridSize().x, physicsEngine.getGridSize().y});
//    std::cout << "Grid size: " << physicsEngine.getGridSize().x << " " << physicsEngine.getGridSize().y << std::endl;
    // Draw balls
    float red = 0.0, green = 0.0, blue = 0.0;

    for (int i = 0; i < physicsEngine.getNumberOfGameObjects(); ++i) {
            glm::vec3 color = glm::vec3(0.0, 0.0, 0.0);

            int gridPosition = physicsEngine.getObjectGridPosition(i);
            if (gridPosition == -1) {
                color = glm::vec3(1.0, 0.0, 0.0);
            }

            double x = physicsEngine.getGameObjectXPosition(i);
            double y = physicsEngine.getGameObjectYPosition(i);
            double radius = physicsEngine.getGameObjectRadius(i);
            drawCircle(x,  y, radius, 10, color);
        }

    }




// Timer callback function for animation
void timer(int) {

    // imgui position in world
    glutPostRedisplay();
    glutTimerFunc(16, timer, 0);  // 60 frames per second

}

void task(float dt) {
    physicsEngine.update(dt, 8);
}

void periodicTask(std::atomic<bool>& stopFlag) {
    while (!stopFlag) {
        // Call the task
        float dt = 0.001f;
        // measure time

        auto start = std::chrono::high_resolution_clock::now();

        task(0.015f);
        auto end = std::chrono::high_resolution_clock::now();
        // Calculate the duration
        std::chrono::duration<double> elapsed = end - start;
        physicsTime = elapsed;
        // Output the duration in seconds


        // Sleep for 10 milliseconds
        auto sleepTime = std::chrono::milliseconds(16) - std::chrono::duration_cast<std::chrono::milliseconds>(elapsed);
        if (sleepTime.count() > 0)
            std::this_thread::sleep_for(sleepTime);
    }
}

int getObjectHovered(double xMousePos, double yMousePos) {
    for (int i = 0; i < physicsEngine.getNumberOfGameObjects(); i++) {
        double xPos = physicsEngine.getGameObjectXPosition(i);
        double yPos = physicsEngine.getGameObjectYPosition(i);
        double radius = physicsEngine.getGameObjectRadius(i);
        if (glm::distance(glm::vec2(xPos, yPos), glm::vec2(xMousePos, yMousePos)) < radius) {
            return i;
        }
    }
    return -1;
}


void generateObjects(double radius, int numObjects) {
    double distance = 2 * 1.05 * radius;
    int numOfObjectsInRow = static_cast<int>((2.0 - distance) / distance);
    int numOfRows = numObjects / numOfObjectsInRow;
    int leftOver = numObjects % numOfObjectsInRow;
    if (numOfRows >= numOfObjectsInRow) {
        numOfRows = numOfObjectsInRow;
        leftOver = 0;
    }

    std::vector<glm::vec2> gameObjectsPositions = {};
    for (int i = 0; i < numOfRows; i++) {
        for (int j = 0; j < numOfObjectsInRow; j++) {
            double xPos = -(1.0 - distance) + distance * j;
            double yPos = -(1.0 - distance) + distance * i;
           // gameObjectsPositions.push_back(glm::vec2(xPos, yPos));
            // sleep for 10 milliseconds

            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            physicsEngine.addGameObject(xPos, yPos, 0.0, 0.0, radius);
        }
    }
    for (int i = 0; i < leftOver ; i++) {
        double xPos = -(1.0 - distance) + distance * i;
        double yPos = -(1.0 - distance) + distance * numOfRows;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));

        physicsEngine.addGameObject(xPos, yPos, 0.0, 0.0, radius);
        //gameObjectsPositions.push_back(glm::vec2(xPos, yPos));
    }

    //physicsEngine.addGameObjects(gameObjectsPositions, radius);

}

int main(int argc, char** argv) {
    glutInit(&argc, argv);
    glutInitWindowSize(width, height);
    glutInitWindowPosition(100, 100);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
    glutCreateWindow("2D Particle System DEMO");
    glutDisplayFunc(MainLoopStep);
    glutTimerFunc(0, timer, 0);
    glClearColor(0.0, 0.0, 0.0, 1.0);  // Black background
    gluOrtho2D(-1.0, 1.0, -1.0, 1.0);  // Set the coordinate system


    outputFile.open("../results.txt");


    // Check if the file is successfully opened
    if (!outputFile.is_open()) {
        std::cerr << "Error opening the file!" << std::endl;
        return 1; // Return an error code
    }
    std::atomic<bool> stopFlag(false);  // Stop flag to control thread execution
    // set nubmer of threads


    // Start the thread and call the function every 10ms
    std::thread worker(periodicTask, std::ref(stopFlag));



    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();
    //ImGui::StyleColorsLight();

    // Setup Platform/Renderer backends
    ImGui_ImplGLUT_Init();
    ImGui_ImplOpenGL2_Init();
    ImGui_ImplGLUT_InstallFuncs();

    glutMainLoop();


    // Cleanup
    ImGui_ImplOpenGL2_Shutdown();
    ImGui_ImplGLUT_Shutdown();
    ImGui::DestroyContext();
    outputFile.close();


    // Stop the worker thread
    stopFlag = true;

    // Wait for the worker thread to finish
    worker.join();

    return 0;
}
float gravity = 0.0;
bool debug = true;
bool useGpu = false;
int objectsToAdd = 10000;
float objectSize = 0.008;
void MainLoopStep()

{

    // Start the Dear ImGui frame
    ImGui_ImplOpenGL2_NewFrame();
    ImGui_ImplGLUT_NewFrame();
    ImGui::NewFrame();


    ImGuiIO& io = ImGui::GetIO();

    {
        ImGui::Begin("Interactive GUI panel");
        if (ImGui::IsKeyPressed(ImGuiKey_Space)) {

            std::cout << "Pause simulation" << std::endl;
            if (physicsEngine.isSimulationPaused()) {
                physicsEngine.resumeSimulation();
            } else {
                physicsEngine.pauseSimulation();
            }
        }
        if(ImGui::IsMouseClicked(0) && !ImGui::IsWindowHovered() && !ImGui::IsAnyItemHovered()){
            double x = io.MousePos.x;
            double y = io.MousePos.y;
            double xPos = 2.0 * x / width - 1.0;
            double yPos = 1.0 - 2.0 * y / height;
            physicsEngine.addGameObject(xPos, yPos, 0.0, 0.0, objectSize);
        }
        ImGui::SliderFloat("Object size", &objectSize, 0.001, 0.05);
        if(ImGui::SliderInt("Number of objects to add", &objectsToAdd, 0, 1000000)){
        }
        std::string label = "Add " + std::to_string(objectsToAdd) + " objects";
        if(ImGui::Button(label.c_str())){
            std::thread worker(generateObjects, std::ref(objectSize), std::ref(objectsToAdd));
            worker.detach();
        }
        gravity = physicsEngine.getGravity().y;
        if(ImGui::SliderFloat("Gravity Y", &gravity, -1, 1)){
            physicsEngine.setGravity(glm::vec2(0, gravity));
        }
        int threadCount = physicsEngine.getThreadCount();
        if(ImGui::SliderInt("Thread count", &threadCount, 1, 16)){
            physicsEngine.setThreadCount(threadCount);
        }

        if (ImGui::Checkbox("Use GPU", &useGpu)){
            physicsEngine.setUseGPU(useGpu);
        }

        if(ImGui::Button("Clear")){
            physicsEngine.clearGameObjects();
        }

        double fps = ImGui::GetIO().Framerate;
        ImGui::Text("Number of game objects: %d", physicsEngine.getNumberOfGameObjects());
        ImGui::Text("FPS: %.1f", fps);
        ImGui::Text("Physics time: %.3f ms", physicsTime.count() * 1000);

        // DEBUG
        ImGui::Checkbox("Debug", &debug);
        if (debug) {
            double x = io.MousePos.x;
            double y = io.MousePos.y;
            double xPos = 2.0 * x / width - 1.0;
            double yPos = 1.0 - 2.0 * y / height;
            int gridPos = physicsEngine.getGridPositionFromWorldPosition(xPos, yPos);
            ImGui::Text("Mouse position x= %.3f y= %.3f, Grid coordinates x=%d, y=%d (index = %d)", xPos, yPos, gridPos / physicsEngine.getGridSize(), gridPos % physicsEngine.getGridSize(), gridPos);
            int objectHovered = getObjectHovered(xPos, yPos);
            if (objectHovered != -1) {
                double hoveredX = physicsEngine.getGameObjectXPosition(objectHovered);
                double hoveredY = physicsEngine.getGameObjectYPosition(objectHovered);
                double hoveredRadius = physicsEngine.getGameObjectRadius(objectHovered);
                int gridPosObj = physicsEngine.getObjectGridPosition(objectHovered);
                ImGui::Text("Hovered %d: x=%.3f y=%.3f, r=%.3f, grid=%d", objectHovered, hoveredX, hoveredY, hoveredRadius, gridPosObj);
                // if (ImGui::IsMouseDown(0) && !ImGui::IsWindowHovered() && !ImGui::IsAnyItemHovered()) {
                //     physicsEngine.setGameObjectXPosition(objectHovered, xPos);
                //     physicsEngine.setGameObjectYPosition(objectHovered, yPos);
                // }
            }
            int* adjacentCells = new int[8];
            int adjacentCount = physicsEngine.calculateAdjacentCellsPublicTest(gridPos, adjacentCells);
            ImGui::Text("Cell: %d, adjacent count: %d", gridPos, adjacentCount);
            for (int i = 0; i < adjacentCount; i++) {
                ImGui::Text("Adjacent cell %d: %d", i, adjacentCells[i]);
            }

        }

        outputFile << "FPS:" << physicsTime.count() * 1000 << ";GRID_SIZE:" << physicsEngine.getGridSize() <<";PARTICLE_COUNTER:" << physicsEngine.getNumberOfGameObjects()  << ";THREAD_COUNT:"<< threadCount << std::endl;


        ImGui::End();

    }


    // Rendering
    ImGui::Render();
    glViewport(0, 0, (GLsizei)io.DisplaySize.x, (GLsizei)io.DisplaySize.y);
    glClearColor(0.5f, 0.5f, 0.5f, 1.f);
    glClear(GL_COLOR_BUFFER_BIT);

    drawGrid(physicsEngine.getGridSize());

    display();


    ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());


    glutSwapBuffers();

}