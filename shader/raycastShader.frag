#version 430 core

layout (std430, binding = 0) buffer gridCBuf {
    uint C[];
}

layout (std430, binding = 1) buffer gridSBuf {
    float S[];
}

layout (std430, binding = 2) buffer IMBuf {
    uint IM[];
}

layout (std430, binding = 3) buffer DMBuf {
    float DM[];
}

uniform mat3 intrinsic_matrix_inv;
uniform vec3 T_Camera;
uniform mat3 R_Camera_inv;
uniform int K;

const double INFINITY = 3.4e38;

int getVoxelIndex(ivec3 pos);
uvec3 unpackPixel(uint p);
uint packPixel(uvec3 c);

void main() {
    uvec2 coord = uvec2(gl_FragCoord.xy);
    int index = K * coord.y + coord.x;
    vec3 v = R_Camera_inv * intrinsic_matrix_inv * vec3(gl_FragCoord.xy, 1);
    vec3 p = vec3(T_Camera.xyz);

    p.x += K / 2;
    p.y += K / 2;
    int dir = 0;
    int beg = 0;
    int step = 1;
    if (fabs(v[1]) > fabs(v[0]) && fabs(v[1]) > fabs(v[2])) {
        dir = 1;
    }
    else if (fabs(v[2]) > fabs(v[0]) && fabs(v[2]) > fabs(v[1])) {
        dir = 2;
    }
    if (v[dir] < 0) {
        beg = K - 1;
        step = -1;
    }
    int dir1 = (dir + 1) % 3, dir2 = (dir + 2) % 3;

    uvec3 old_C, new_C;
    float old_S = INFINITY, new_S;
    vec3 old_real_p, new_real_p;
    for (int i = beg; i >= 0 && i < K; i++) {
        float t;
        vec3 p0, p1, p2;
        int voxelIndex[3] = {-1, -1, -1};
        t = (float(i) - p[dir]) / v[dir];
        if (t < 0) {
            continue;
        }
        p0[dir] = float(i);
        p0[dir1] = p[dir1] + t * v[dir1];
        p0[dir2] = p[dir2] + t * v[dir2];
        voxelIndex[0] = getVoxelIndex(p0);
        t = (i + 1 - p[dir]) / v[dir];
        if (t < 0) {
            continue;
        }
        p2[dir] = float(i);
        p2[dir1] = p[dir1] + t * v[dir1];
        p2[dir2] = p[dir2] + t * v[dir2];
        voxelIndex[2] = getVoxelIndex(p2);
        if ((min(p0[dir1], p2[dir1]) - int(max(p0[dir1], p2[dir1])) < 0) && 
            (min(p0[dir2], p2[dir2]) - int(max(p0[dir2], p2[dir2])) < 0)) {
            p1[dir] = float(i);
            p1[dir1] = (v[dir1] > 0) ? min(p0[dir1], p2[dir1]) : max(p0[dir1], p2[dir1]);
            p1[dir2] = (v[dir2] > 0) ? min(p0[dir2], p2[dir2]) : max(p0[dir2], p2[dir2]);
            voxelIndex[1] = getVoxelIndex(p1);
        }
        float cnt = 0;
        new_S = 0;
        new_C = uvec3(0, 0, 0);
        for (int j = 0; j < 3; j++) {
            if (voxelIndex[i] != -1) {
                new_C = cnt / (cnt + 1) * new_C + 1.0 / (cnt + 1) * unpackPixel(C[voxelIndex[i]]);
                new_S = cnt / (cnt + 1) * new_S + 1.0 / (cnt + 1) * S[voxelIndex[i]];
                cnt++;
            }
        }
        if (cnt == 0) {
            continue;
        }
        new_real_p[dir] = i + 0.5;
        new_real_p[dir1] = (p0[dir1] + p2[dir1]) / 2;
        new_real_p[dir2] = (p2[dir2] + p2[dir2]) / 2;
        if (new_S <= 0) {
            if (new_S == 0) {
                C = new_C;
            }
            else {
                float tmp_newS = -1 / new_S, tmp_oldS = 1 / old_S;
                float tmp_sumS = tmp_newS + tmp_oldS;
                float para_new = tmp_newS / tmp_sumS;
                float para_old = tmp_oldS / tmp_sumS;
                IM[index] = packPixel(para_new * new_C + para_new * old_C);
                new_real_p = para_new * new_real_p + para_old * old_real_p;
            }
            DM[index] = new_real_p - vec3(K / 2, K / 2, 0);
        }
        else {
            old_C = new_C;
            old_S = new_S;
            old_real_p = new_real_p;
        }
    }
}

int getVoxelIndex(ivec3 pos) {
    return pos.x * K * K + pos.y * K + pos.z;
}

uvec3 unpackPixel(uint p) {
    uint r = p & 255;
    uint g = (p >> 8) & 255;
    uint b = (p >> 16) & 255;
    return uvec3(r, g, b);
}

uint packPixel(uvec3 c) {
    return uint((255 << 24) + (p.b << 16) + (p.g << 8) + p.r);
}