#include "Grid.h"

Grid::Grid(int K) {
    C = new GLbyte***[K];
    S = new GLfloat**[K];
    W = new int**[K];
    for (int i = 0; i < K; i++) {
        C[i] = new GLbyte**[K];
        S[i] = new GLfloat*[K];
        W[i] = new int*[K];
        for (int j = 0; j < K; j++) {
            C[i][j] = new GLbyte*[K];
            S[i][j] = new GLfloat[K];
            W[i][j] = new int[K];
            for (int k = 0; k < K; k++) {
                C[i][j][k] = new GLbyte[K];
                GLbyte *vC = C[i][j][k];
                vC[0] = vC[1] = vC[2] = 0;
                vC[3] = 1;
                S[i][j][k] = INFINITY;
                W[i][j][k] = 0;
            }
        }
    }

    for (int i = 0; i < K; i++) {
        for (int j = 0; j < K; j++) {
            for (int k = 0; k < K; k++) {
                GLbyte *vC = C[i][j][k];
                GLfloat &vS = S[i][j][k];
                vC[0] = 255;
                vS = 0.1;
            }
        }
    }

    for (int i = K / 4; i < K * 3 / 4; i++) {
        for (int j = K / 4; j < K * 3 / 4; j++) {
            for (int k = K / 4; k < K * 3 / 4; k++) {
                GLbyte *vC = C[i][j][k];
                GLfloat &vS = S[i][j][k];
                vC[0] = 255;
                vS = -0.1;
            }
        }
    }
}

Grid::~Grid() {
    for (int i = 0; i < K; i++) {
        for (int j = 0; j < K; j++) {
            for (int k = 0; k < K; k++) {
                delete[] C[i][j][k];
            }
            delete[] C[i][j];
            delete[] S[i][j];
            delete[] W[i][j];
        }
        delete[] C[i];
        delete[] S[i];
        delete[] W[i];
    }
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
    IM = cv::Mat(IMG_SIZE, CV_8UC4);
    DM = cv::Mat(IMG_SIZE, CV_32FC1, cv::Scalar::all(-1));

    cv::Mat intrinsic_matrix_inv = intrinsic_matrix.inv();

    cv::Mat R_Camera(Tv, cv::Rect(0, 0, 3, 3));
    cv::Mat R_Camera_inv = R_Camera.inv();
    cv::Mat T_Camera = -R_Camera_inv * cv::Mat(Tv, cv::Rect(3, 0, 1, 3));

    Shader shader("../shader/vShader.vs", "../shader/raycastShader.frag");

    // put Grid_C, Grid_S, intrinsic_matrix_inv, cameraPoseT, camereaPoseR into buffer
    GLuint CBuf, SBuf, IMBuf, DMBuf;
    const GLuint CBuf_index = 0;
    const GLuint SBuf_index = 1;
    const GLuint IMBuf_index = 2;
    const GLuint DMBuf_index = 3;
    // copy Grid_C
    glGenBuffers(1, &CBuf);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, CBuf);
    glBufferData(GL_SHADER_STORAGE_BUFFER, K * K * K * sizeof(GLuint), C, GL_DYNAMIC_COPY); // TODO: check if other type other than dynamic copy
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, CBuf_index, CBuf);
    // copy Grid_S
    glGenBuffers(1, &SBuf);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, SBuf);
    glBufferData(GL_SHADER_STORAGE_BUFFER, K * K * K * sizeof(GLfloat), S, GL_DYNAMIC_COPY);
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
    intrinsic_matrix_inv.convertTo(intrinsic_matrix_inv, CV_32FC1);
    T_Camera.convertTo(T_Camera, CV_32FC1);
    R_Camera_inv.convertTo(R_Camera_inv, CV_32FC1);
    GLint intrinsic_matrix_inv_loc = glGetUniformLocation(shader.program, "intrinsic_matrix_inv");
    /*
     * TODO: parameter GL_TRUE in glUniformMatrix3fv indicates that I wanna transpose matrix when
     * passing it into shader. But Does it really necessary? If the program went wrong, go check it.
     */
    glUniformMatrix3fv(intrinsic_matrix_inv_loc, 1, GL_TRUE, (GLfloat*)intrinsic_matrix_inv.data);
    GLint T_Camera_loc = glGetUniformLocation(shader.program, "T_Camera");
    glUniform1fv(T_Camera_loc, 1, (GLfloat*)T_Camera.data);
    GLint R_Camera_inv_loc = glGetUniformLocation(shader.program, "R_Camera_inv");
    glUniformMatrix3fv(R_Camera_inv_loc, 1, GL_TRUE, (GLfloat*)R_Camera_inv.data);
    GLint K_loc = glGetUniformLocation(shader.program, "K");
    glUniform1i(K_loc, (GLint)K);

    gpuRun(shader);

    // get Image and Depth map back from GPU memory
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, IMBuf);
    glGetBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, FRAME_WIDTH * FRAME_HEIGHT * sizeof(GLuint), IM.data);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, DMBuf);
    glGetBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, FRAME_WIDTH * FRAME_HEIGHT * sizeof(GLfloat), DM.data);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
}


// lagacy
//Old_Grid::Old_Grid(int K):K(K) {
//    voxels = new Voxel**[K];
//    for (int i = 0; i < K; i++) {
//        voxels[i] = new Voxel*[K];
//        for (int j = 0; j < K; j++) {
//            voxels[i][j] = new Voxel[K];
//        }
//    }
//
//    for (int i = 0; i < K; i++) {
//        for (int j = 0; j < K; j++) {
//            for (int k = 0; k < K; k++) {
//                voxels[i][j][k].C = cv::Vec3b(255, 0, 0);
//                voxels[i][j][k].S = 0.1;
//            }
//        }
//    }
//
//    for (int i = K / 4; i < K * 3 / 4; i++) {
//        for (int j = K / 4; j < K * 3 / 4; j++) {
//            for (int k = K / 4; k < K * 3 / 4; k++) {
//                voxels[i][j][k].C = cv::Vec3b(255, 0, 0);
//                voxels[i][j][k].S = -0.1;
//            }
//        }
//    }
//
//}
//
//Old_Grid::~Old_Grid() {
//    for (int i = 0; i < K; i++) {
//        for (int j = 0; j < K; j++) {
//            delete[] voxels[i][j];
//        }
//        delete[] voxels[i];
//    }
//    delete[] voxels;
//}
//
//inline Voxel * Old_Grid::getVoxelByCoordinate(double x, double y, double z) {
//    if (x >= K || y >= K || z >= K || x < 0 || y < 0 || z < 0) {
//        return nullptr;
//    }
//    else {
//        return &voxels[(int)x][(int)y][(int)z];
//    }
//}
//
//inline Voxel * Old_Grid::getVoxelByCoordinate(cv::Vec3d v) {
//    return getVoxelByCoordinate(v[0], v[1], v[2]);
//}
//
//bool Old_Grid::getColorAndPointByRay(cv::Vec3d p, cv::Vec3d v, cv::Vec3b & C, cv::Mat & worldP) {
//    p[0] += K / 2; // move the grid corner to the original point of world's coordinate
//    p[1] += K / 2;
//
//    // the direction to traverse layers of grid. dir = 0 for x, dir = 1 for y, dir = 2 for z
//    int dir = 0;
//    int beg = 0;
//    int step = 1;
//    if (fabs(v[1]) > fabs(v[0]) && fabs(v[1]) > fabs(v[2])) {
//        dir = 1;
//    }
//    else if (fabs(v[2]) > fabs(v[0]) && fabs(v[2]) > fabs(v[1])) {
//        dir = 2;
//    }
//    if (v[dir] < 0) {
//        beg = K - 1;
//        step = -1;
//    }
//    int dir1 = (dir + 1) % 3, dir2 = (dir + 2) % 3;
//
//    cv::Vec3b old_C, new_C;
//    double old_S = INFINITY, new_S;
//    cv::Mat old_real_p, new_real_p;
//    for (int i = beg; i >= 0 && i < K; i++) {
//        double t;
//        cv::Vec3d p0, p1, p2;
//        Voxel* voxel[3] = { nullptr, nullptr, nullptr };
//        t = (i - p[dir]) / v[dir];
//        if (t < 0) {
//            continue;
//        }
//        p0[dir] = (double)i;
//        p0[dir1] = p[dir1] + t * v[dir1];
//        p0[dir2] = p[dir2] + t * v[dir2];
//        voxel[0] = getVoxelByCoordinate(p0);
//        t = (i + 1 - p[dir]) / v[dir];
//        if (t < 0) {
//            continue;
//        }
//        p2[dir] = (double)i;
//        p2[dir1] = p[dir1] + t * v[dir1];
//        p2[dir2] = p[dir2] + t * v[dir2];
//        voxel[2] = getVoxelByCoordinate(p2);
//        if (min(p0[dir1], p2[dir1]) - (int)max(p0[dir1], p2[dir1]) < 0) {
//            if (min(p0[dir2], p2[dir2]) - (int)max(p0[dir2], p2[dir2]) < 0) {
//                p1[dir] = i;
//                p1[dir1] = (v[dir1] > 0) ? min(p0[dir1], p2[dir1]) : max(p0[dir1], p2[dir1]);
//                p1[dir2] = (v[dir2] > 0) ? min(p0[dir2], p2[dir2]) : max(p0[dir2], p2[dir2]);
//                voxel[1] = getVoxelByCoordinate(p1);
//            }
//        }
//        int cnt = 0;
//        new_S = 0;
//        new_C = cv::Vec3b::all(0);
//        for (int j = 0; j < 3; j++) {
//            if (voxel[j]) {
//                new_C = (double)cnt / (cnt + 1) * new_C + (double)1 / (cnt + 1) * voxel[j]->C;
//                new_S = (double)cnt / (cnt + 1) * new_S + (double)1 / (cnt + 1) * voxel[j]->S;
//                cnt++;
//            }
//        }
//        if (cnt == 0) {
//            continue;
//        }
//        cv::Vec3d tmp;
//        tmp[dir] = i + 0.5;
//        tmp[dir1] = (p0[dir1] + p2[dir1]) / 2;
//        tmp[dir2] = (p2[dir2] + p2[dir2]) / 2;
//        new_real_p = cv::Mat(tmp);
//        if (new_S <= 0) {
//            if (new_S == 0) {
//                C = new_C;
//            }
//            else {
//                double tmp_newS = -1 / new_S, tmp_oldS = 1 / old_S;
//                double tmp_sumS = tmp_newS + tmp_oldS;
//                double para_new = tmp_newS / tmp_sumS;
//                double para_old = tmp_oldS / tmp_sumS;
//                C = para_new * new_C + para_new * old_C;
//                new_real_p = para_new * new_real_p + para_old * old_real_p;
//            }
//            worldP = new_real_p - cv::Mat(cv::Vec3d(K / 2, K / 2, 0));
//            return true;
//        }
//        else {
//            old_C = new_C;
//            old_S = new_S;
//            old_real_p = new_real_p;
//        }
//    }
//    return false;
//}