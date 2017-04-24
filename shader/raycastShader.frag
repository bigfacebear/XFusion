#version 430 core

layout (std430, binding = 0) buffer gridCBuf {
    uint C[];
};

layout (std430, binding = 1) buffer gridSBuf {
    float S[];
};

layout (std430, binding = 2) buffer gridWBuf {
    float W[];
};

layout (std430, binding = 3) buffer IMBuf {
    uint IM[];
};

layout (std430, binding = 4) buffer DMBuf {
    float DM[];
};

layout (std430, binding = 5) buffer WMBuf {
    float WM[];
};

uniform mat3 intrinsic_matrix_inv;
uniform vec3 T_Camera;
uniform mat3 R_Camera_inv;
uniform mat3 R_Camera;
uniform int K;

out vec3 color;

int getVoxelIndex(vec3 pos);
int getVoxelIndex(ivec3 pos);
uvec3 unpackPixel(uint p);
uint packPixel(uvec3 c);

void main() {
    ivec2 coord = ivec2(gl_FragCoord.x, 240 - gl_FragCoord.y - 1);
    const int index = 320 * coord.y + coord.x;
    vec3 v = R_Camera_inv * intrinsic_matrix_inv * vec3(coord.xy, 1);
    vec3 p = -R_Camera_inv * T_Camera;

    p.x += K / 2;
    p.y += K / 2;
    int dir = 0;
    int beg = 0;
    int step = 1;
    if (abs(v[1]) > abs(v[0]) && abs(v[1]) > abs(v[2])) {
        dir = 1;
    }
    else if (abs(v[2]) > abs(v[0]) && abs(v[2]) > abs(v[1])) {
        dir = 2;
    }
    if (v[dir] < 0) {
        beg = K - 1;
        step = -1;
    }
    int dir1 = (dir + 1) % 3, dir2 = (dir + 2) % 3;

    uvec3 old_C = uvec3(0, 0, 0), new_C;
    float old_S = 0.0, new_S;
    float old_W = 0.0, new_W;
    vec3 old_real_p = vec3(0.0, 0.0, 0.0), new_real_p;
    bool flag = false;
    for (int i = beg; i >= 0 && i < K; i += step) {
        float t = (float(i + 0.5) - p[dir]) / v[dir];
        if (t < 0) {
            continue;
        }
        new_real_p = vec3(float(i + 0.5), p[dir1] + t * v[dir1], p[dir2] + t * v[dir2]);
        int voxelIndex = getVoxelIndex(new_real_p);
        if (voxelIndex == -1) {
            color = vec3(0.4, 0.5, 0.6);
            continue;
        }
        new_C = unpackPixel(C[voxelIndex]);
        new_S = S[voxelIndex];
        new_W = W[voxelIndex];
        if (new_S <= 0) {
            if (new_S == 0 || !flag) {
                IM[index] = packPixel(new_C);
                WM[index] = new_W;
            }
            else {
                float tmp_newS = -1 / new_S, tmp_oldS = 1 / old_S;
                float tmp_sumS = tmp_newS + tmp_oldS;
                float para_new = tmp_newS / tmp_sumS;
                float para_old = tmp_oldS / tmp_sumS;
                IM[index] = packPixel(uvec3(para_new * new_C + para_old * old_C));
                WM[index] = para_new * new_W + para_old * old_W;
                new_real_p = para_new * new_real_p + para_old * old_real_p;
            }
            if (WM[index] <= 0) {
                WM[index] = 1;
            }
            vec3 new_real_p = new_real_p - vec3(K / 2, K / 2, 0);
            vec3 camera_p = R_Camera * new_real_p  + T_Camera;
            DM[index] = camera_p.z;
            color = vec3(1,1,1);
            return;
        }
        else {
            old_C = new_C;
            old_S = new_S;
            old_W = new_W;
            old_real_p = new_real_p;
            flag = true;
        }
    }
    IM[index] = packPixel(uvec3(0,0,0));
    DM[index] = 1e8;
    WM[index] = 0;
    color = vec3(0.2, 0.3, 0.3);
}

int getVoxelIndex(vec3 pos) {
    return getVoxelIndex(ivec3(pos));
}

int getVoxelIndex(ivec3 pos) {
    if (pos.x >= K || pos.y >= K || pos.z >= K || pos.x < 0 || pos.y < 0 || pos.z < 0) {
        return -1;
    }
    return pos.x * K * K + pos.y * K + pos.z;
}

uvec3 unpackPixel(uint p) {
    uint r = uint(p & uint(255));
    uint g = uint((p >> 8) & uint(255));
    uint b = uint((p >> 16) & uint(255));
    return uvec3(r, g, b);
}

uint packPixel(uvec3 c) {
    return uint((255 << 24) + (c.b << 16) + (c.g << 8) + c.r);
}