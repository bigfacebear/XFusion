#version 430 core

layout (std430, binding = 0) buffer gridCBuf {
    uint C[];
};

layout (std430, binding = 1) buffer gridSBuf {
    float S[];
};

layout (std430, binding = 2) buffer IMBuf {
    uint IM[];
};

layout (std430, binding = 3) buffer DMBuf {
    float DM[];
};

uniform mat3 intrinsic_matrix_inv;
uniform vec3 T_Camera;
uniform mat3 R_Camera_inv;
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
    vec3 p = vec3(T_Camera.xyz);

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
    vec3 old_real_p = vec3(0.0, 0.0, 0.0), new_real_p;
    bool flag = false;
    for (int i = beg; i >= 0 && i < K; i += step) {
        float t;
        vec3 p0, p1, p2;
        int voxelIndex[3] = {-1, -1, -1};
        voxelIndex[0] = voxelIndex[1] = voxelIndex[2] = -1;
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
            if (voxelIndex[j] != -1) {
                new_C = uvec3(cnt / (cnt + 1) * vec3(new_C) + 1.0 / (cnt + 1) * vec3(unpackPixel(C[voxelIndex[j]])));
                new_S = cnt / (cnt + 1) * new_S + 1.0 / (cnt + 1) * S[voxelIndex[j]];
                cnt += 1;
            }
        }
        if (cnt == 0) {
            continue;
        }
        new_real_p[dir] = i + 0.5;
        new_real_p[dir1] = (p0[dir1] + p2[dir1]) / 2;
        new_real_p[dir2] = (p0[dir2] + p2[dir2]) / 2;
        if (new_S <= 0) {
            if (new_S == 0 || !flag) {
                IM[index] = packPixel(new_C);
            }
            else {
                float tmp_newS = -1 / new_S, tmp_oldS = 1 / old_S;
                float tmp_sumS = tmp_newS + tmp_oldS;
                float para_new = tmp_newS / tmp_sumS;
                float para_old = tmp_oldS / tmp_sumS;
                IM[index] = packPixel(uvec3(para_new * new_C + para_old * old_C));
                new_real_p = para_new * new_real_p + para_old * old_real_p;
            }
            DM[index] = (new_real_p - vec3(K / 2, K / 2, 0)).z;

            color = vec3(1, 1, 1);

            return;
        }
        else {
            old_C = new_C;
            old_S = new_S;
            old_real_p = new_real_p;
            flag = true;
        }
    }
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