//
// Created by bfb on 17-4-21.
//

#include "definition.h"

extern GLuint DRAW_VAO;

void gpuRun(Shader &shader) {
    shader.use();
    glBindVertexArray(DRAW_VAO);
    glDrawArrays(GL_TRIANGLES, 0, 6);
    glBindVertexArray(0);
}


void getShaderBufferObjectDataToImage(GLuint buffer, size_t pixelSize, cv::Mat& img) {
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, buffer);
    glGetBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, img.cols * img.rows * pixelSize, img.data);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
}
