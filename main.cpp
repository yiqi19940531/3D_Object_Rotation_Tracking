/**************************************************************************************************
 **************************************************************************************************
 
     BSD 3-Clause License (https://www.tldrlegal.com/l/bsd3)
     
     Copyright (c) 2014 Andrés Solís Montero <http://www.solism.ca>, All rights reserved.
     
     
     Redistribution and use in source and binary forms, with or without modification,
     are permitted provided that the following conditions are met:
     
     1. Redistributions of source code must retain the above copyright notice,
        this list of conditions and the following disclaimer.
     2. Redistributions in binary form must reproduce the above copyright notice,
        this list of conditions and the following disclaimer in the documentation
        and/or other materials provided with the distribution.
     3. Neither the name of the copyright holder nor the names of its contributors
        may be used to endorse or promote products derived from this software
        without specific prior written permission.
     
     THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
     AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
     IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
     ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
     LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
     DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
     LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
     THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
     OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
     OF THE POSSIBILITY OF SUCH DAMAGE.
 
 **************************************************************************************************
 **************************************************************************************************/

#include "objectdetector.h"
using namespace viva;

#include "ObjectRotator/ObjectRotator.h"

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtx/euler_angles.hpp>
#include <glm/gtx/norm.hpp>
using namespace glm;

#include <stdio.h>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <sstream>

#include <stdlib.h>

void pushImages_Cube(vector<string> &);
ObjectRotator *initRotator();
Processor *initDetector(int cameraId, ObjectRotator*);

void render(glm::mat4 &cubeTransform);
void teardown();

// OpenGL context
GLFWwindow* window;
GLuint vaoId;
GLuint programID;
GLuint mvpID;
GLuint vertexbuffer;
GLuint colorbuffer;

int main(int argc, char **argv)
{
    Processor *detector;

    int cameraId = 0;
    if (argc > 1)
    {
        cameraId = std::stoi(argv[1]);
    }

    printf("--- Detection 3D With Object Rotator  ---\n");
    printf("--- Using camera with id %d\n", cameraId);
    printf("\n");

    ObjectRotator *rotator = initRotator();
    if (!rotator)
    {
        printf("!!! Error Initializing Object Rotator !!!\n");
    }
    else
    {
        rotator->setRotation(0);
        rotator->renderCube();
    }

    detector = initDetector(cameraId, rotator);

    detector->run();

    teardown();
    delete detector;
    delete rotator;

    return 0;
}

/*
 *  3D object recognition and localization using an uncalibrated camera
 */
Processor *initDetector(int cameraId, ObjectRotator *rotator)
{
    vector<string> images;
    pushImages_Cube(images);

    Feat::Code feat = Feat::RFAST;
    Desc::Code desc = Desc::BRIEF;

    //Ptr<Input> input;
    Ptr<Input> input;
    Ptr<Output> output = new VideoOutput("output.avi", Size(640, 480));
    Ptr<ProcessFrame> process = new ObjectDetector(images, feat, desc, rotator);

    Processor *processor = new Processor();
    processor->setProcess(process);

    if (!cameraId)
    {
        processor->setOutput(output);
    }
    else
    {
        input = new CameraInput(cameraId, Size(640, 640));
        processor->setInput(input);
    }

    processor->setOutput(output);

    return processor;
}

void pushImages_Cube(vector<string> &images)
{
    int imgSelection = 0;  // 0 for cube, 1 for cube 640 480, 2 for star cube, 3 for star cube 640 480, 4 for car,
    // 5 for eight faces

    switch(imgSelection){
        case 0:
            images.emplace_back("/home/yiqi/Desktop/detection3D/input/binarycode/1.jpg");
            images.emplace_back("/home/yiqi/Desktop/detection3D/input/binarycode/2.jpg");
            images.emplace_back("/home/yiqi/Desktop/detection3D/input/binarycode/3.jpg");
            images.emplace_back("/home/yiqi/Desktop/detection3D/input/binarycode/4.jpg");
            images.emplace_back("/home/yiqi/Desktop/detection3D/input/binarycode/5.jpg");
            images.emplace_back("/home/yiqi/Desktop/detection3D/input/binarycode/6.jpg");
            break;
        case 1:
            images.emplace_back("/home/yiqi/Desktop/detection3D/input/binarycode640480/1.jpg");
            images.emplace_back("/home/yiqi/Desktop/detection3D/input/binarycode640480/2.jpg");
            images.emplace_back("/home/yiqi/Desktop/detection3D/input/binarycode640480/3.jpg");
            images.emplace_back("/home/yiqi/Desktop/detection3D/input/binarycode640480/4.jpg");
            images.emplace_back("/home/yiqi/Desktop/detection3D/input/binarycode640480/5.jpg");
            images.emplace_back("/home/yiqi/Desktop/detection3D/input/binarycode640480/6.jpg");
            break;
        case 2:
            images.emplace_back("/home/yiqi/Desktop/detection3D/input/starcube/1.jpg");
            images.emplace_back("/home/yiqi/Desktop/detection3D/input/starcube/2.jpg");
            images.emplace_back("/home/yiqi/Desktop/detection3D/input/starcube/3.jpg");
            images.emplace_back("/home/yiqi/Desktop/detection3D/input/starcube/4.jpg");
            break;
        case 3:
            images.emplace_back("/home/yiqi/Desktop/detection3D/input/starcube640480/1.jpg");
            images.emplace_back("/home/yiqi/Desktop/detection3D/input/starcube640480/2.jpg");
            images.emplace_back("/home/yiqi/Desktop/detection3D/input/starcube640480/3.jpg");
            images.emplace_back("/home/yiqi/Desktop/detection3D/input/starcube640480/4.jpg");
            break;
        case 4:
            images.emplace_back("/home/yiqi/Desktop/detection3D/input/car/1.jpg");
            images.emplace_back("/home/yiqi/Desktop/detection3D/input/car/2.jpg");
            images.emplace_back("/home/yiqi/Desktop/detection3D/input/car/3.jpg");
            images.emplace_back("/home/yiqi/Desktop/detection3D/input/car/4.jpg");
            break;
        case 5:
            images.emplace_back("/home/yiqi/Desktop/detection3D/input/dragon/1.jpg");
            images.emplace_back("/home/yiqi/Desktop/detection3D/input/dragon/2.jpg");
            images.emplace_back("/home/yiqi/Desktop/detection3D/input/dragon/3.jpg");
            images.emplace_back("/home/yiqi/Desktop/detection3D/input/dragon/4.jpg");


        default:
            break;
    }




}

GLuint LoadShaders(const char *vertex_file_path, const char *fragment_file_path)
{
    GLuint VertexShaderID = glCreateShader(GL_VERTEX_SHADER);
    GLuint FragmentShaderID = glCreateShader(GL_FRAGMENT_SHADER);

    std::string VertexShaderCode;
    std::ifstream VertexShaderStream(vertex_file_path, std::ios::in);
    if(VertexShaderStream.is_open())
    {
        std::stringstream sstr;
        sstr << VertexShaderStream.rdbuf();
        VertexShaderCode = sstr.str();
        VertexShaderStream.close();
    }
    else
    {
        printf("Impossible to open %s. Are you in the right directory ? Don't forget to read the FAQ !\n", vertex_file_path);
        getchar();
        return 0;
    }

    // Read the Fragment Shader code from the file
    std::string FragmentShaderCode;
    std::ifstream FragmentShaderStream(fragment_file_path, std::ios::in);
    if(FragmentShaderStream.is_open()){
        std::stringstream sstr;
        sstr << FragmentShaderStream.rdbuf();
        FragmentShaderCode = sstr.str();
        FragmentShaderStream.close();
    }

    GLint Result = GL_FALSE;
    int InfoLogLength;


    // Compile Vertex Shader
    printf("Compiling shader : %s\n", vertex_file_path);
    char const * VertexSourcePointer = VertexShaderCode.c_str();
    glShaderSource(VertexShaderID, 1, &VertexSourcePointer , NULL);
    glCompileShader(VertexShaderID);

    // Check Vertex Shader
    glGetShaderiv(VertexShaderID, GL_COMPILE_STATUS, &Result);
    glGetShaderiv(VertexShaderID, GL_INFO_LOG_LENGTH, &InfoLogLength);
    if ( InfoLogLength > 0 ){
        std::vector<char> VertexShaderErrorMessage(InfoLogLength+1);
        glGetShaderInfoLog(VertexShaderID, InfoLogLength, NULL, &VertexShaderErrorMessage[0]);
        printf("%s\n", &VertexShaderErrorMessage[0]);
    }

    // Compile Fragment Shader
    printf("Compiling shader : %s\n", fragment_file_path);
    char const * FragmentSourcePointer = FragmentShaderCode.c_str();
    glShaderSource(FragmentShaderID, 1, &FragmentSourcePointer , NULL);
    glCompileShader(FragmentShaderID);

    // Check Fragment Shader
    glGetShaderiv(FragmentShaderID, GL_COMPILE_STATUS, &Result);
    glGetShaderiv(FragmentShaderID, GL_INFO_LOG_LENGTH, &InfoLogLength);
    if ( InfoLogLength > 0 ){
        std::vector<char> FragmentShaderErrorMessage(InfoLogLength+1);
        glGetShaderInfoLog(FragmentShaderID, InfoLogLength, NULL, &FragmentShaderErrorMessage[0]);
        printf("%s\n", &FragmentShaderErrorMessage[0]);
    }

    // Link the program
    printf("Linking program\n");
    GLuint ProgramID = glCreateProgram();
    glAttachShader(ProgramID, VertexShaderID);
    glAttachShader(ProgramID, FragmentShaderID);
    glLinkProgram(ProgramID);

    // Check the program
    glGetProgramiv(ProgramID, GL_LINK_STATUS, &Result);
    glGetProgramiv(ProgramID, GL_INFO_LOG_LENGTH, &InfoLogLength);
    if ( InfoLogLength > 0 ){
        std::vector<char> ProgramErrorMessage(InfoLogLength+1);
        glGetProgramInfoLog(ProgramID, InfoLogLength, NULL, &ProgramErrorMessage[0]);
        printf("%s\n", &ProgramErrorMessage[0]);
    }

    glDetachShader(ProgramID, VertexShaderID);
    glDetachShader(ProgramID, FragmentShaderID);

    glDeleteShader(VertexShaderID);
    glDeleteShader(FragmentShaderID);

    return ProgramID;
}

ObjectRotator *initRotator()
{
    if( !glfwInit() )
    {
        fprintf( stderr, "Failed to initialize GLFW\n" );
        getchar();
        return nullptr;
    }

    glfwWindowHint(GLFW_SAMPLES, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // To make MacOS happy; should not be needed
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    window = glfwCreateWindow( 800, 600, "Object Rotator", NULL, NULL); //Create the cube window Yiqi
    if( window == nullptr){
        fprintf( stderr, "Failed to open GLFW window.\n");
        getchar();
        glfwTerminate();
        return nullptr;
    }

    glfwMakeContextCurrent(window);

    glewExperimental = true; // Needed for core profile
    if (glewInit() != GLEW_OK) {
        fprintf(stderr, "Failed to initialize GLEW\n");
        getchar();
        glfwTerminate();
        return nullptr;
    }

    glClearColor(0.0f, 0.0f, 0.4f, 0.0f);

    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS); // Accept fragment if it closer to the camera than the former one

    glGenVertexArrays(1, &vaoId);
    glBindVertexArray(vaoId);

    programID = LoadShaders("shaders/vertexShader.vs", "shaders/fragmentShader.fs");

    mvpID = glGetUniformLocation(programID, "MVP");

    static const GLfloat g_vertex_buffer_data[] = {
            // -X Triangle 1
            -1.0f,-1.0f,-1.0f,
            -1.0f,-1.0f, 1.0f,
            -1.0f, 1.0f, 1.0f,

            // -Z Triangle 1
            1.0f, 1.0f,-1.0f,
            -1.0f,-1.0f,-1.0f,
            -1.0f, 1.0f,-1.0f,

            // -Y Triangle 1
            1.0f,-1.0f, 1.0f,
            -1.0f,-1.0f,-1.0f,
            1.0f,-1.0f,-1.0f,

            // -Z Triangle 2
            1.0f, 1.0f,-1.0f,
            1.0f,-1.0f,-1.0f,
            -1.0f,-1.0f,-1.0f,

            // -X Triangle 2
            -1.0f,-1.0f,-1.0f,
            -1.0f, 1.0f, 1.0f,
            -1.0f, 1.0f,-1.0f,

            // -Y Triangle 2
            1.0f,-1.0f, 1.0f,
            -1.0f,-1.0f, 1.0f,
            -1.0f,-1.0f,-1.0f,

            // +Z Triangle 1
            -1.0f, 1.0f, 1.0f,
            -1.0f,-1.0f, 1.0f,
            1.0f,-1.0f, 1.0f,

            // +X Triangle 1
            1.0f, 1.0f, 1.0f,
            1.0f,-1.0f,-1.0f,
            1.0f, 1.0f,-1.0f,

            // +X Triangle 2
            1.0f,-1.0f,-1.0f,
            1.0f, 1.0f, 1.0f,
            1.0f,-1.0f, 1.0f,

            // +Y Triangle 1
            1.0f, 1.0f, 1.0f,
            1.0f, 1.0f,-1.0f,
            -1.0f, 1.0f,-1.0f,

            // +Y Triangle 2
            1.0f, 1.0f, 1.0f,
            -1.0f, 1.0f,-1.0f,
            -1.0f, 1.0f, 1.0f,

            // +Z Triangle 2
            1.0f, 1.0f, 1.0f,
            -1.0f, 1.0f, 1.0f,
            1.0f,-1.0f, 1.0f
    };

    static const GLfloat g_color_buffer_data[] = {
            // -X Triangle 1
            0.0f,    0.5f,    0.0f,
            0.0f,    0.5f,    0.0f,
            0.0f,    0.5f,    0.0f,

            // -Z Triangle 1
            0.5f,    0.0f,    0.0f,
            0.5f,    0.0f,    0.0f,
            0.5f,    0.0f,    0.0f,

            // -Y Triangle 1
            0.597f,  0.770f,  0.761f,
            0.559f,  0.436f,  0.730f,
            0.359f,  0.583f,  0.152f,

            // -Z Triangle 2
            0.5f,    0.0f,    0.0f,
            0.5f,    0.0f,    0.0f,
            0.5f,    0.0f,    0.0f,

            // -X Triangle 2
            0.0f,    0.5f,    0.0f,
            0.0f,    0.5f,    0.0f,
            0.0f,    0.5f,    0.0f,

            // -Y Triangle 2
            0.676f,  0.977f,  0.133f,
            0.971f,  0.572f,  0.833f,
            0.140f,  0.616f,  0.489f,

            // +Z Triangle 1
            0.5f,    0.5f,    0.0f,
            0.5f,    0.5f,    0.0f,
            0.5f,    0.5f,    0.0f,

            // +X Triangle 1
            0.0f,    0.5f,    0.5f,
            0.0f,    0.5f,    0.5f,
            0.0f,    0.5f,    0.5f,

            // +X Triangle 2
            0.0f,    0.5f,    0.5f,
            0.0f,    0.5f,    0.5f,
            0.0f,    0.5f,    0.5f,

            // +Y Triangle 1
            0.722f,  0.645f,  0.174f,
            0.302f,  0.455f,  0.848f,
            0.225f,  0.587f,  0.040f,

            // +Y Triangle 2
            0.517f,  0.713f,  0.338f,
            0.053f,  0.959f,  0.120f,
            0.393f,  0.621f,  0.362f,

            // +Z Triangle 2
            0.5f,    0.5f,    0.0f,
            0.5f,    0.5f,    0.0f,
            0.5f,    0.5f,    0.0f,
    };

    glGenBuffers(1, &vertexbuffer);
    glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(g_vertex_buffer_data), g_vertex_buffer_data, GL_STATIC_DRAW);

    glGenBuffers(1, &colorbuffer);
    glBindBuffer(GL_ARRAY_BUFFER, colorbuffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(g_color_buffer_data), g_color_buffer_data, GL_STATIC_DRAW);

    return new ObjectRotator(&render);
}

void render(glm::mat4 &cubeTransform)
{


    static const glm::mat4 projection = glm::perspective(glm::radians(45.0f), 4.0f / 3.0f, 0.1f, 100.0f);
    static const glm::mat4 view       = glm::lookAt(
            glm::vec3(0,0,-10),
            glm::vec3(0,0,0),
            glm::vec3(0,1,0)
    );

    glm::mat4 mvp = projection * view * cubeTransform;

    //glm::mat4 mvp = projection * view * transform;
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glUseProgram(programID);

    glUniformMatrix4fv(mvpID, 1, GL_FALSE, &mvp[0][0]);

    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void *) 0);

    glEnableVertexAttribArray(1);
    glBindBuffer(GL_ARRAY_BUFFER, colorbuffer);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, (void *) 0);

    glDrawArrays(GL_TRIANGLES, 0, 12 * 3);

    glDisableVertexAttribArray(0);
    glDisableVertexAttribArray(1);

    glfwSwapBuffers(window);
    glfwPollEvents();
}

void teardown()
{
    glDeleteBuffers(1, &vertexbuffer);
    glDeleteBuffers(1, &colorbuffer);
    glDeleteProgram(programID);
    glDeleteVertexArrays(1, &vaoId);

    glfwTerminate();
}
