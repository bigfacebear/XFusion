#include "Grid.h"

Grid::Grid(int K):K(K) {
    C = new GLbyte[K * K * K * 4]{0};
    S = new GLfloat[K * K * K]{INFINITY};
    W = new int[K * K * K]{0};

    for (int i = 0; i < K; i++) {
        for (int j = 0; j < K; j++) {
            for (int k = 0; k < K; k++) {
                GLbyte *tmpC = getC(i, j, k);
                tmpC[3] = (GLbyte)255;
            }
        }
    }

    for (int i = 0; i < K; i++) {
        for (int j = 0; j < K; j++) {
            for (int k = 0; k < K; k++) {
                GLbyte *tmpC = getC(i, j, k);
                tmpC[2] = (GLbyte)255;
                getS(i, j, k) = 0.1;
            }
        }
    }

    for (int i = K / 4; i < K * 3 / 4; i++) {
        for (int j = K / 4; j < K * 3 / 4; j++) {
            for (int k = K / 4; k < K * 3 / 4; k++) {
                GLbyte *tmpC = getC(i, j, k);
                tmpC[2] = (GLbyte)255;
                getS(i, j, k) = (GLfloat)-0.1;
            }
        }
    }
}

Grid::~Grid() {
    delete[] C;
    delete[] S;
    delete[] W;
}

void Grid::getImageAndDepthFromViewPoint(
        // input
        PoseMatx Tv, // camera pose
        cv::Mat intrinsic_matrix, // camera's intrinsic parameters
        // output
        cv::Mat &IM, // RGBA image, data in uchar
        cv::Mat &DM  // depth map, data in GLfloat
) {
    static Shader shader("../shader/vShader.vs", "../shader/raycastShader.frag");

    IM = cv::Mat(IMG_SIZE, CV_8UC4, cv::Scalar::all(0));
    DM = cv::Mat(IMG_SIZE, CV_32FC1, cv::Scalar::all(0));

    cv::Mat intrinsic_matrix_inv = intrinsic_matrix.inv();

    cv::Mat R_Camera(Tv, cv::Rect(0, 0, 3, 3));
    cv::Mat R_Camera_inv = R_Camera.inv();
    cv::Mat T_Camera = cv::Mat(Tv, cv::Rect(3, 0, 1, 3)).clone();

    shader.use();

    // put Grid_C, Grid_S, intrinsic_matrix_inv, cameraPoseT, camereaPoseR into buffer
    GLuint CBuf, SBuf, IMBuf, DMBuf;
    const GLuint CBuf_index = 0;
    const GLuint SBuf_index = 1;
    const GLuint IMBuf_index = 2;
    const GLuint DMBuf_index = 3;
    // copy Grid_C
    glGenBuffers(1, &CBuf);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, CBuf);
    glBufferData(GL_SHADER_STORAGE_BUFFER, K * K * K * sizeof(GLuint), (void*)C, GL_STATIC_READ); // TODO: check if other type other than dynamic copy
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, CBuf_index, CBuf);
    // copy Grid_S
    glGenBuffers(1, &SBuf);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, SBuf);
    glBufferData(GL_SHADER_STORAGE_BUFFER, K * K * K * sizeof(GLfloat), (void*)S, GL_STATIC_READ);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, SBuf_index, SBuf);
    // allocate space for IM
    glGenBuffers(1, &IMBuf);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, IMBuf);
    glBufferData(GL_SHADER_STORAGE_BUFFER, FRAME_WIDTH * FRAME_HEIGHT * sizeof(GLuint), nullptr, GL_DYNAMIC_COPY);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, IMBuf_index, IMBuf);
    // allocate space for DM
    glGenBuffers(1, &DMBuf);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, DMBuf);
    glBufferData(GL_SHADER_STORAGE_BUFFER, FRAME_WIDTH * FRAME_HEIGHT * sizeof(GLfloat), nullptr, GL_DYNAMIC_COPY);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, DMBuf_index, DMBuf);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
    // pass other three matrix into GPU memory
    // convert data of matrices to float
    intrinsic_matrix_inv.convertTo(intrinsic_matrix_inv, cv::DataType<float>::type);
    T_Camera.convertTo(T_Camera, cv::DataType<float>::type);
    R_Camera_inv.convertTo(R_Camera_inv, cv::DataType<float>::type);
    GLint intrinsic_matrix_inv_loc = glGetUniformLocation(shader.program, "intrinsic_matrix_inv");
    glUniformMatrix3fv(intrinsic_matrix_inv_loc, 1, GL_TRUE, (GLfloat*)intrinsic_matrix_inv.data);
    GLint T_Camera_loc = glGetUniformLocation(shader.program, "T_Camera");
    glUniform3fv(T_Camera_loc, 1, (GLfloat*)T_Camera.data);
    GLint R_Camera_loc = glGetUniformLocation(shader.program, "R_Camera");
    glUniformMatrix3fv(R_Camera_loc, 1, GL_TRUE, (GLfloat*)R_Camera.data);
    GLint R_Camera_inv_loc = glGetUniformLocation(shader.program, "R_Camera_inv");
    glUniformMatrix3fv(R_Camera_inv_loc, 1, GL_TRUE, (GLfloat*)R_Camera_inv.data);
    GLint K_loc = glGetUniformLocation(shader.program, "K");
    glUniform1i(K_loc, (GLint)K);

    gpuRun();

    // get Image and Depth map back from GPU memory
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, IMBuf);
    glGetBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, FRAME_WIDTH * FRAME_HEIGHT * sizeof(GLuint), (void*)IM.data);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, DMBuf);
    glGetBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, FRAME_WIDTH * FRAME_HEIGHT * sizeof(GLfloat), (void*)DM.data);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);

    glDeleteBuffers(1, &CBuf);
    glDeleteBuffers(1, &SBuf);
    glDeleteBuffers(1, &IMBuf);
    glDeleteBuffers(1, &DMBuf);
}