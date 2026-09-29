// pbrt is Copyright(c) 1998-2020 Matt Pharr, Wenzel Jakob, and Greg Humphreys.
// The pbrt source code is licensed under the Apache License, Version 2.0.
// SPDX: Apache-2.0

#include <pbrt/util/gui.h>

#include <pbrt/options.h>
#ifdef PBRT_BUILD_GPU_RENDERER
#include <pbrt/gpu/util.h>
#endif  // PBRT_BUILD_GPU_RENDERER
#include <pbrt/util/error.h>
#include <pbrt/util/image.h>
#include <pbrt/util/parallel.h>
#include <pbrt/util/progressreporter.h>

#if defined(PBRT_WITH_PATH_GUIDING)
#include <pbrt/wavefront/guidingoptions.h>
#endif

#include <iostream>

#ifndef M_PI
#define M_PI 3.14159265358979323846264f 
#endif

#define GL_CHECK(call)                                                   \
    do {                                                                 \
        call;                                                            \
        if (GLenum err = glGetError(); err != GL_NO_ERROR)               \
            LOG_FATAL("GL error: %s for " #call, getGLErrorString(err)); \
    } while (0)

#define GL_CHECK_ERRORS()                                     \
    do {                                                      \
        if (GLenum err = glGetError(); err != GL_NO_ERROR)    \
            LOG_FATAL("GL error: %s", getGLErrorString(err)); \
    } while (0)

namespace pbrt {

const char *getGLErrorString(GLenum error) {
    switch (error) {
    case GL_NO_ERROR:
        return "No error";
    case GL_INVALID_ENUM:
        return "Invalid enum";
    case GL_INVALID_VALUE:
        return "Invalid value";
    case GL_INVALID_OPERATION:
        return "Invalid operation";
    case GL_OUT_OF_MEMORY:
        return "Out of memory";
    default:
        return "Unknown GL error";
    }
}

static void glfwErrorCallback(int error, const char *desc) {
    LOG_ERROR("GLFW [%d]: %s", error, desc);
}

void GUI::keyboardCallback(GLFWwindow *window, int key, int scan, int action, int mods) {
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
        glfwSetWindowShouldClose(window, GLFW_TRUE);

    auto doKey = [&](int k, char ch) {
        if (key == k) {
            if (action == GLFW_PRESS)
                keysDown.insert(ch);
            else if (action == GLFW_RELEASE) {
                if (auto iter = keysDown.find(ch); iter != keysDown.end())
                    keysDown.erase(iter);
            }
        }
    };

    doKey(GLFW_KEY_A, 'a');
    doKey(GLFW_KEY_D, 'd');
    doKey(GLFW_KEY_S, 's');
    doKey(GLFW_KEY_W, 'w');
    doKey(GLFW_KEY_Q, 'q');
    doKey(GLFW_KEY_E, 'e');

#if defined(PBRT_WITH_PATH_GUIDING)
    doKey(GLFW_KEY_G, 'g');
#endif

    doKey(GLFW_KEY_B, (mods & GLFW_MOD_SHIFT) ? 'B' : 'b');
    doKey(GLFW_KEY_C, 'c');
    doKey(GLFW_KEY_EQUAL, '=');
    doKey(GLFW_KEY_MINUS, '-');

    doKey(GLFW_KEY_LEFT, 'L');
    doKey(GLFW_KEY_RIGHT, 'R');
    doKey(GLFW_KEY_UP, 'U');
    doKey(GLFW_KEY_DOWN, 'D');

    if (key == GLFW_KEY_R && action == GLFW_PRESS &&
        glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS)
        recordFrames = !recordFrames;
    else
        doKey(GLFW_KEY_R, 'r');
}

bool GUI::processMouse() {
    bool needsReset = false;
    double amount = 1.f;
    if (!pressed)
        return false;

    if(xoffset == 0 && yoffset == 0)
        return false;

    ImGuiIO& io = ImGui::GetIO();
    if (io.WantCaptureMouse)
        return false;

    movingFromCamera.processMouse(xoffset, yoffset);
    xoffset = 0;
    yoffset = 0;
    needsReset = true;

    return needsReset;
}

bool GUI::process() {
    bool needsReset = false;
    needsReset |= processKeys();
    needsReset |= processMouse();
    needsReset |= processOptions();
    return needsReset;
}

bool GUI::processOptions() {
    bool needsReset = false;

    if(RendererOptions->useNEE != guiRendererOptions.useNEE) {
        RendererOptions->useNEE = guiRendererOptions.useNEE;
        RendererOptions->update = true;
        needsReset = true;
    }
    if(RendererOptions->maxDepth != guiRendererOptions.maxDepth) {
        RendererOptions->maxDepth = guiRendererOptions.maxDepth;
        RendererOptions->update = true;
        needsReset = true;
    }
    if(RendererOptions->minRRDepth != guiRendererOptions.minRRDepth) {
        RendererOptions->minRRDepth = guiRendererOptions.minRRDepth;
        RendererOptions->update = true;
        needsReset = true;
    }

#if defined(PBRT_WITH_PATH_GUIDING)
    if(GuidingOptions->enableGuiding != guiGuidingOptions.enableGuiding) {
        GuidingOptions->enableGuiding = guiGuidingOptions.enableGuiding;
        GuidingOptions->update = true;
        needsReset = true;
    }
    if(GuidingOptions->guideSurface != guiGuidingOptions.guideSurface) {
        GuidingOptions->guideSurface = guiGuidingOptions.guideSurface;
        GuidingOptions->update = true;
        needsReset = true;
    }
    if(GuidingOptions->guideVolume != guiGuidingOptions.guideVolume) {
        GuidingOptions->guideVolume = guiGuidingOptions.guideVolume;
        GuidingOptions->update = true;
        needsReset = true;
    }
#endif
    return needsReset;
}

bool GUI::processKeys() {
    bool needsReset = false;

    if (keysDown.find('a') != keysDown.end()) {
        movingFromCamera.processKey('a', moveScale);
        needsReset = true;
    }

    if (keysDown.find('d') != keysDown.end()) {
        movingFromCamera.processKey('d', moveScale);
        needsReset = true;
    }

    if (keysDown.find('s') != keysDown.end()) {
        movingFromCamera.processKey('s', moveScale);
        needsReset = true;
    }

    if (keysDown.find('w') != keysDown.end()) {
        movingFromCamera.processKey('w', moveScale);
        needsReset = true;
    }

    if (keysDown.find('q') != keysDown.end()) {
        movingFromCamera.processKey('q', moveScale);
        needsReset = true;
    }

    if (keysDown.find('e') != keysDown.end()) {
        movingFromCamera.processKey('e', moveScale);
        needsReset = true;
    }
/*
    auto handleNeedsReset = [&](char key, std::function<Transform(Transform)> update) {
        if (keysDown.find(key) != keysDown.end()) {
            movingFromCamera.processKey(key, moveScale);
            //movingFromCamera = update(movingFromCamera);
            needsReset = true;
        }
    };

    handleNeedsReset(
        'a', [&](Transform t) { return t * Translate(Vector3f(-moveScale, 0, 0)); });
    handleNeedsReset(
        'd', [&](Transform t) { return t * Translate(Vector3f(moveScale, 0, 0)); });
    handleNeedsReset(
        's', [&](Transform t) { return t * Translate(Vector3f(0, 0, -moveScale)); });
    handleNeedsReset(
        'w', [&](Transform t) { return t * Translate(Vector3f(0, 0, moveScale)); });
    handleNeedsReset(
        'q', [&](Transform t) { return t * Translate(Vector3f(0, -moveScale, 0)); });
    handleNeedsReset(
        'e', [&](Transform t) { return t * Translate(Vector3f(0, moveScale, 0)); });
    handleNeedsReset('L',
                     [&](Transform t) { return t * Rotate(-.5f, Vector3f(0, 1, 0)); });
    handleNeedsReset('R',
                     [&](Transform t) { return t * Rotate(.5f, Vector3f(0, 1, 0)); });
    handleNeedsReset('U',
                     [&](Transform t) { return t * Rotate(-.5f, Vector3f(1, 0, 0)); });
    handleNeedsReset('D',
                     [&](Transform t) { return t * Rotate(.5f, Vector3f(1, 0, 0)); });
    handleNeedsReset('r', [&](Transform t) { return Transform(); });
*/

    if (keysDown.find('g') != keysDown.end()) {
        keysDown.erase(keysDown.find('g'));
        showGUI = !showGUI;
    }

    // No reset needed for these.
    if (keysDown.find('c') != keysDown.end()) {
        keysDown.erase(keysDown.find('c'));
        printCameraTransform = true;
    }
    if (keysDown.find('b') != keysDown.end()) {
        keysDown.erase(keysDown.find('b'));
        exposure *= 1.125f;
    }
    if (keysDown.find('B') != keysDown.end()) {
        keysDown.erase(keysDown.find('B'));
        exposure /= 1.125f;
    }
    if (keysDown.find('=') != keysDown.end()) {
        keysDown.erase(keysDown.find('='));
        moveScale *= 2;
    }
    if (keysDown.find('-') != keysDown.end()) {
        keysDown.erase(keysDown.find('-'));
        moveScale *= 0.5;
    }

    return needsReset;
}

static void glfwKeyCallback(GLFWwindow* window, int key, int scan, int action, int mods) {
    GUI* gui = (GUI*)glfwGetWindowUserPointer(window);
    gui->keyboardCallback(window, key, scan, action, mods);
}

void GUI::mouseButtonCallback(GLFWwindow* window, int button, int action, int mods) {
    if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
        pressed = true;
        glfwGetCursorPos(window, &lastX, &lastY);
    }
    if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_RELEASE) {
        pressed = false;
    }
}

void GUI::Initialize() {
    if (!glfwInit())
        LOG_FATAL("Unable to initialize GLFW");
}

Point2i GUI::GetResolution() {
    auto monitor = glfwGetPrimaryMonitor();
    auto videoMode = glfwGetVideoMode(monitor);
    return Point2i(videoMode->width, videoMode->height);
}

static void glfwMouseButtonCallback(GLFWwindow* window, int button, int action,
                                    int mods) {
    GUI* gui = (GUI*)glfwGetWindowUserPointer(window);
    gui->mouseButtonCallback(window, button, action, mods);
}

static void glfwCursorPosCallback(GLFWwindow* window, double xpos, double ypos) {
    GUI* gui = (GUI*)glfwGetWindowUserPointer(window);
    gui->cursorPosCallback(window, xpos, ypos);
}

void GUI::cursorPosCallback(GLFWwindow* window, double xpos, double ypos) {
    xoffset = xpos - lastX;
    yoffset = lastY - ypos;
    lastX = xpos;
    lastY = ypos;
}

void GUI::DrawOptions() {
    ImGui::Begin("Render settings");
    ImGui::SeparatorText("Path Tracer:");
    ImGui::Checkbox("Use NEE:", &guiRendererOptions.useNEE);
    ImGui::SliderInt("Max depth", &guiRendererOptions.maxDepth, 1, 30);
    ImGui::SliderInt("Min RR depth", &guiRendererOptions.minRRDepth, 1, 30);
#if defined(PBRT_WITH_PATH_GUIDING)
    ImGui::SeparatorText("Guiding:");
    ImGui::Checkbox("Enable:", &guiGuidingOptions.enableGuiding);
    ImGui::Checkbox("Surface guiding:", &guiGuidingOptions.guideSurface);
    ImGui::Checkbox("Volume guiding:", &guiGuidingOptions.guideVolume);
#endif
    ImGui::End();
}



GUI::GUI(std::string title, Vector2i resolution, Bounds3f sceneBounds, Transform cameraFromWorld)
    : resolution(resolution), movingFromCamera(cameraFromWorld) {

    guiRendererOptions = GetRendererOptions();
#if defined(PBRT_WITH_PATH_GUIDING)
    guiGuidingOptions = GetGuidingOptions();
#endif
    moveScale = Length(sceneBounds.Diagonal()) / 1000.f;

    glfwSetErrorCallback(glfwErrorCallback);
    if (Options->fullscreen) {
        window = glfwCreateWindow(resolution.x, resolution.y, "pbrt", glfwGetPrimaryMonitor(), NULL);
    } else {
        window = glfwCreateWindow(resolution.x, resolution.y, "pbrt", NULL, NULL);
    }

    if (!window) {
        glfwTerminate();
        LOG_FATAL("Unable to create GLFW window");
    }
    glfwSetKeyCallback(window, glfwKeyCallback);
    glfwSetMouseButtonCallback(window, glfwMouseButtonCallback);
    glfwSetCursorPosCallback(window, glfwCursorPosCallback);


    glfwSetWindowUserPointer(window, this);
    glfwMakeContextCurrent(window);

    // Initialize ImGui
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    //ImGui::StyleColorsDark();
    ImGui::Spectrum::StyleColorsSpectrum();
    /*
    ImGuiStyle& style = ImGui::GetStyle(); 
    style.WindowBorderSize = 0.5f;
    style.FrameRounding = 5.f;
    style.GrabRounding = 3.f; 
    style.ChildRounding = 3.f; 
    style.FrameBorderSize = 0.0f;
    */
    //setStyle();
    //ImGuiStyle& style = ImGui::GetStyle();
    //ImVec4 windowBgColor = style.Colors[ImGuiCol_WindowBg];
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 130");


/*
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
*/
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
        LOG_FATAL("gladLoadGLLoader failed");

#ifdef PBRT_BUILD_GPU_RENDERER
    if (Options->useGPU)
        cudaFramebuffer = new CUDAOutputBuffer<RGB>(resolution.x, resolution.y);
    else
#endif  // PBRT_BUILD_GPU_RENDERER
        cpuFramebuffer = new RGB[resolution.x * resolution.y];
}

GUI::~GUI() {
#ifdef PBRT_BUILD_GPU_RENDERER
    delete cudaFramebuffer;
#endif  // PBRT_BUILD_GPU_RENDERER
    delete[] cpuFramebuffer;

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();
}

void GUI::UpdateFPS() {

    if (frameCounter > 0) {
        float fps = 1.f / frameTimer.ElapsedSeconds();
        float alpha = 1.f/float(frameCounter);
        
        avgFPS = (1.f-alpha)*avgFPS + alpha*fps;
        
        if(fpsUpdateTimer.ElapsedSeconds() > 0.5f)
        {
            std::stringstream s; 
            s << "Fps: "<< avgFPS;
            glfwSetWindowTitle(window, s.str().c_str());
            fpsUpdateTimer = Timer();
        }
        frameTimer = Timer();
    }
    frameCounter = std::min(frameCounter+1,32);
}

DisplayState GUI::RefreshDisplay() {
    int width, height;
    glfwGetFramebufferSize(window, &width, &height);
    int windowWidth, windowHeight;
    glfwGetWindowSize(window, &windowWidth, &windowHeight);
    GL_CHECK(glViewport(0, 0, width, height));
    float pixelScales[2] = {(float)width / (float)windowWidth,
                            (float)height / (float)windowHeight};

#ifdef PBRT_BUILD_GPU_RENDERER
    if (Options->useGPU)
        cudaFramebuffer->Draw(width, height);
    else
#endif  // PBRT_BUILD_GPU_RENDERER
    {
        GL_CHECK(glEnable(GL_FRAMEBUFFER_SRGB));
        GL_CHECK(glRasterPos2f(-1, 1));
        GL_CHECK(glPixelZoom(pixelScales[0], -pixelScales[1]));
        GL_CHECK(
            glDrawPixels(resolution.x, resolution.y, GL_RGB, GL_FLOAT, cpuFramebuffer));
    }

    if(showGUI) {
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        DrawOptions();

        // Main Window
        ImGui::Render();

        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    }
    glfwSwapBuffers(window);
    glfwPollEvents();

    UpdateFPS();

    if (recordFrames) {
        const RGB *fb = nullptr;
#ifdef PBRT_BUILD_GPU_RENDERER
        if (cudaFramebuffer)
            fb = cudaFramebuffer->GetReadbackPixels();
        else
#endif
            fb = cpuFramebuffer;

        if (fb) {
            Image image(PixelFormat::Float, {width, height}, {"R", "G", "B"});
            std::memcpy(image.RawPointer({0, 0}), fb, width * height * sizeof(RGB));

            RunAsync(
                [](Image image, int frameNumber) {
                    // TODO: set metadata for e.g. current camera position...
                    ImageMetadata metadata;
                    image.Write(StringPrintf("pbrt-frame%05d.exr", frameNumber),
                                metadata);
                    return 0;  // FIXME: RunAsync() doesn't like lambdas that return
                               // void..
                },
                std::move(image), frameNumber);

            ++frameNumber;
        }
#ifdef PBRT_BUILD_GPU_RENDERER
        if (cudaFramebuffer)
            cudaFramebuffer->StartAsynchronousReadback();
#endif
    }

    if (glfwWindowShouldClose(window))
        return DisplayState::EXIT;
    else if (process())
        return DisplayState::RESET;
    else
        return DisplayState::NONE;
}


MovingCamera::MovingCamera(Transform cameraFromWorld) {
    m_cameraFromWorld = cameraFromWorld;
    ExtractLookAt(Inverse(cameraFromWorld));
}

void MovingCamera::ExtractLookAt(Transform transform){

    init_origin = transform(Point3f(0.f, 0.f, 0.f));
    init_front = transform(Vector3f(0.f, 0.f, 1.f));
    init_up = transform(Vector3f(0.f, 1.f, 0.f));

    origin = init_origin;
    front = init_front;
    if(init_up[0] > init_up[1]) {
        if(init_up[0] > init_up[2]) {
            world_up = Vector3f(1.f, 0.f, 0.f);
            cameraUp = X_UP;
        } else {
            world_up = Vector3f(0.f, 0.f, 1.f);
            cameraUp = Z_UP;
        }
    } else {
        if(init_up[1] > init_up[2]) {
            world_up = Vector3f(0.f, 1.f, 0.f);
            cameraUp = Y_UP;
        } else {
            world_up = Vector3f(0.f, 0.f, 1.f);
            cameraUp = Z_UP;
        }
    }

    switch (cameraUp) {
        case X_UP: {
            break;
        }
        case Y_UP: {
            init_pitch = std::acos(init_front[1]) * 180.f/ M_PI;
            init_yaw   = std::atan2(init_front[2], init_front[0]) * 180.f/ M_PI;
            break;
        }
        case Z_UP: {
            init_pitch = std::acos(init_front[2]) * 180.f/ M_PI;
            init_yaw   = std::atan2(init_front[1], init_front[0]) * 180.f/ M_PI;
            break;
        }        
    }
    
    std::cout << "cameraUp = " << cameraUp << std::endl;

    yaw = init_yaw;
    pitch = init_pitch;
}

void MovingCamera::processKey(char key, float moveScale) {
    moveScale *= 5.f;
    switch (key){
        case 'w': {
            origin += front * moveScale;
            break;
        }
        case 'a': {
            origin -= Normalize(Cross(front, world_up)) * moveScale;
            break;
        }
        case 's': {
            origin -= front * moveScale;
            break;
        }
        case 'd': {
            origin += Normalize(Cross(front, world_up)) * moveScale;
            break;
        }
        case 'q': {
            origin += world_up * moveScale;
            break;
        }
        case 'e': {
            origin -= world_up * moveScale;
            break;
        }
        case 'r': {
            origin = init_origin;
            front = init_front;
            yaw = init_yaw;
            pitch = init_pitch;
            break;
        }
        default: {
            break;
        }
    }
}

void MovingCamera::processMouse(Float xoffset, Float yoffset){
    
    switch (cameraUp) {
        case X_UP: {
            break;
        }
        case Y_UP: {
            yaw += xoffset * 2.0f;
            pitch -= yoffset;
            front[0] = std::cos(yaw * (M_PI / 180.f)) * std::sin(pitch * (M_PI / 180.f));
            front[1] = std::cos(pitch * (M_PI / 180.f));
            front[2] = std::sin(yaw * (M_PI / 180.f)) * std::sin(pitch * (M_PI / 180.f));
            break;
        }
        case Z_UP: {
                yaw -= xoffset * 2.0f;
                pitch -= yoffset;
                front[0] = std::cos(yaw * (M_PI / 180.f)) * std::sin(pitch * (M_PI / 180.f));
                front[1] = std::sin(yaw * (M_PI / 180.f)) * std::sin(pitch * (M_PI / 180.f));
                front[2] = std::cos(pitch * (M_PI / 180.f));
            break;
        }        
    }
}

Transform MovingCamera::GetTransform() const {
    //std::cout << m_cameraFromWorld * Inverse(LookAt(origin, origin+front,world_up)) << std::endl;
    return m_cameraFromWorld * Inverse(Scale(-1.f, 1.f, 1.f) * LookAt(origin, origin+front,world_up));
}
}