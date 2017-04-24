//
// Created by bfb on 17-4-21.
//

#include "XFuser.h"

GLFWwindow *glWindow = nullptr;
GLuint DRAW_VAO;

XFuser::XFuser(cv::Mat intrinsic_matrix): grid(256), framePool(40), intrinsic_matrix(intrinsic_matrix) {
    if (!initOpenGLContext()) {
        std::cout << "Failed to initialize OpenGL context. " << std::endl;
        exit(1);
    }
}

void XFuser::load(const std::string &videoName) {
    ioProcessor.loadVideo(videoName);
}

void XFuser::load(const int cameraHandler) {
}

void XFuser::fuse() {

    bootstrap(framePool, grid, ioProcessor);

    for (cv::Mat Iv = ioProcessor.get(); !Iv.empty(); Iv = ioProcessor.get()) {
        // get a new image from IO, and estimate the pose of the camera
        PoseMatx Tv = framePool.getLatestFrame().T;
        //PoseMatx Tvl = poseEstimator.poseEstimate(Iv, Tv, grid);
        PoseMatx Tvl = poseEstimate(Iv, Tv, grid, intrinsic_matrix);
        PoseMatx T = Tvl * Tv;
        Frame newFrame(Iv, T);

        // Select a keyframe from the frame pool
        Frame &keyFrame = framePool.getKeyFrame(newFrame, intrinsic_matrix);
        // Update the frame pool
        framePool.addFrame(newFrame);

        // Estimate depth map
        // cv::Mat dmap = depthEstimator.depthEstimate(newFrame, keyframe);
        cv::Mat dmap = depthEstimate(newFrame, keyFrame);

        fuseToGrid(grid, T, Iv, dmap, intrinsic_matrix);

        // TODO: live feedback during processing
    }

    // TODO: display the result

}

void XFuser::bootstrap(FramePool & framePool, Grid & grid, IOProcessor & ioProcessor) {
}

PoseMatx XFuser::poseEstimate(cv::Mat &Ii, PoseMatx &Tv, Grid &grid, cv::Mat intrinsic_matrix) {
    cv::Mat IM(IMG_SIZE, CV_8UC4, cv::Scalar::all(0));
    cv::Mat DM(IMG_SIZE, CV_32FC1, cv::Scalar::all(0));
    cv::Mat WM(IMG_SIZE, CV_32FC1, cv::Scalar::all(0));
    grid.getImageAndDepthFromViewPoint(Tv, intrinsic_matrix, IM, DM, WM);

    ceres::Grid2D<uchar, 4> array(Ii.data, 0, Ii.rows, 0, Ii.cols);
    ceres::BiCubicInterpolator<ceres::Grid2D<uchar, 4>> interpolator(array);

    double trans[3];
    double angleAxis[3];

    cv::Mat intrinsic_matrix_inv = intrinsic_matrix.inv();

    ceres::Problem problem;
    for (int u = 0; u < FRAME_WIDTH; u++) {
        for (int v = 0; v < FRAME_HEIGHT; v++) {
            float w = WM.at<float>(v, u);
            if (w == 0) {
                continue;
            }
            float d = DM.at<float>(v, u);
            cv::Mat C = cv::Mat(IM.at<cv::Vec4b>(v, u));
            cv::Mat P = cv::Mat(cv::Vec3f(u, v, 1));
            P = d * intrinsic_matrix_inv * P;
            ceres::CostFunction *costFunction =
                    PoseEstimateFunctor::Create(interpolator, (float*)intrinsic_matrix.data, (float*)P.data, C.data);
            ceres::ScaledLoss lossFunction(NULL, (double)w, ceres::DO_NOT_TAKE_OWNERSHIP);
            problem.AddResidualBlock(costFunction, &lossFunction, trans, angleAxis);
        }
    }

    cv::Mat ret = cv::Mat::eye(4, 4, CV_64FC1);
    cv::Mat RotateMatrix(9, 9, CV_64FC1);
    ceres::AngleAxisToRotationMatrix<double>(angleAxis, (double*)RotateMatrix.data);
    memcpy(ret.data, RotateMatrix.data, 3 * sizeof(double));
    memcpy(ret.data + ret.step[0], RotateMatrix.data + RotateMatrix.step[0], 3 * sizeof(double));
    memcpy(ret.data + 2 * ret.step[0], RotateMatrix.data + 2 * RotateMatrix.step[0], 3 * sizeof(double));
    ret.at<double>(0, 3) = trans[0];
    ret.at<double>(1, 3) = trans[1];
    ret.at<double>(2, 3) = trans[2];
    ret.convertTo(ret, CV_32FC1);

    return ret;
}

cv::Mat XFuser::depthEstimate(const Frame & Ii, const Frame & Ik) {
    return cv::Mat();
}

void XFuser::fuseToGrid(Grid & grid, PoseMatx & T, cv::Mat I, cv::Mat dmap, cv::Mat intrinsic_matrix) {
//    int K = grid.getK();
//    for (int x = 0; x < K; x++) {
//        for (int y = 0; y < K; y++) {
//            for (int z = 0; z < K; z++) {
//                cv::Vec4d p(x - K / 2, y - K / 2, z, 1);  // voxel's world coordinate
//                cv::Vec4d q = cv::Mat(T * cv::Mat(p));  // voxel's camera coordinate
//                cv::Vec3d u_homo = cv::Mat(intrinsic_matrix * cv::Mat(cv::Mat(q), cv::Rect(0, 0, 1, 3)));
//                int u = (int)(u_homo[0] / u_homo[2]);  // voxel's image coordinate.u
//                int v = (int)(u_homo[1] / u_homo[2]);  // voxel's image coordinate.v
//                if (u < 0 || v < 0 || u >= I.cols || v >= I.rows) {
//                    continue;  // this voxel is not on the image
//                }
//                double s = dmap.at<double>(v, u) - q[3] / q[4]; // s = D(u,v) - q.z
//                if (s < -8) {
//                    continue;  // only update visible voxels
//                }
//                if (s > 8) {
//                    s = 8;  // truncate s to 8
//                }
//                cv::Mat c = cv::Mat(I.at<cv::Vec3b>(v, u));
//                c.convertTo(c, CV_64FC3);
//                Voxel &voxel = grid.voxels[x][y][z];
//                voxel.S = (voxel.S * voxel.W + s) / (voxel.W + 1);
//                cv::Mat tmp(voxel.C);
//                tmp.convertTo(tmp, CV_64FC3);
//                tmp = (tmp * voxel.W + c) / (voxel.W + 1);
//                tmp.convertTo(tmp, CV_8UC3);
//                voxel.C = tmp;
//                voxel.W = voxel.W == 50 ? 50 : voxel.W + 1;
//            }
//        }
//    }
}

bool XFuser::initOpenGLContext() {
    if (glfwInit() != GL_TRUE) {
        std::cout << "Failed to initialize GLFW." << std::endl;
        return false;
    }
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);

    glWindow = glfwCreateWindow(FRAME_WIDTH, FRAME_HEIGHT, "OpenGLWindow", nullptr, nullptr);
    if (glWindow == nullptr) {
        std::cout << "Failed to create OpenGL window. ";
        std::cout << "OpenGL version required: 4.3 core profile. " << std::endl;
        return false;
    }
    glfwMakeContextCurrent(glWindow);

    glewExperimental = GL_TRUE;
    if (glewInit() != GLEW_OK) {
        std::cout << "Failed to initialize GLEW.";
        return false;
    }
    glViewport(0, 0, FRAME_WIDTH, FRAME_HEIGHT);

    // init VAO
    GLfloat vertices[] = {
        -1.0f, -1.0f,
        1.0f, 1.0f,
        1.0f, -1.0f,
        -1.0f, -1.0f,
        1.0f, 1.0f,
        -1.0f, 1.0f
    };

    GLuint VBO;
    glGenVertexArrays(1, &DRAW_VAO);
    glGenBuffers(1, &VBO);
    glBindVertexArray(DRAW_VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(GLfloat), (void*)0);
    glEnableVertexAttribArray(0);
    glBindVertexArray(0);

    return true;
}