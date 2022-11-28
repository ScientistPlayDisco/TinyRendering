#include <vector>
#include <cmath>
#include "tgaimage.h"
#include "model.h"
#include "geometry.h"

const TGAColor white = TGAColor(255, 255, 255, 255);
const TGAColor red = TGAColor(255, 0, 0, 255);
Model* model = NULL;
const int width = 800;
const int height = 800;

void line(int x0, int y0, int x1, int y1, TGAImage& image, TGAColor color) {
    bool steep = false;
    if (std::abs(x0 - x1) < std::abs(y0 - y1)) {
        std::swap(x0, y0);
        std::swap(x1, y1);
        steep = true;
    }
    if (x0 > x1) {
        std::swap(x0, x1);
        std::swap(y0, y1);
    }

    for (int x = x0; x <= x1; x++) {
        float t = (x - x0) / (float)(x1 - x0);
        int y = y0 * (1. - t) + y1 * t;
        if (steep) {
            image.set(y, x, color);
        }
        else {
            image.set(x, y, color);
        }
    }
}

//Vec3i crossProduct(Vec2i *pts,Vec2i P) {
//    Vec2i AB(pts[1].x - pts[0].x, pts[1].y - pts[0].y);
//    Vec2i BC(pts[2].x - pts[1].x, pts[2].y - pts[1].y);
//    Vec2i CA(pts[0].x - pts[2].x, pts[0].y - pts[2].y);
//
//    Vec2i AP(P.x-pts[0].x,P.y-pts[0].y);
//    Vec2i BP(P.x - pts[1].x, P.y - pts[1].y);
//    Vec2i CP(P.x - pts[2].x, P.y - pts[2].y);
//
//    return Vec3i(cross(AB,BP),BC^BP,CA^CP);
//}

// 利用重心坐标判断点是否在三角形内部
Vec3f barycentric2D(Vec2i* pts, Vec2i P) {
    Vec3i x(pts[1].x - pts[0].x, pts[2].x - pts[0].x, pts[0].x - P.x);
    Vec3i y(pts[1].y - pts[0].y, pts[2].y - pts[0].y, pts[0].y - P.y);

    // u 向量和 x y 向量的点积为 0，所以 x y 向量叉乘可以得到 u 向量
    Vec3i u = cross(x,y);

    // 由于 A, B, C, P 的坐标都是 int 类型，所以 u.z 必定是 int 类型，取值范围为 ..., -2, -1, 0, 1, 2, ...
    // 所以 u.z 绝对值小于 1 意味着三角形退化了，直接舍弃
    if (std::abs(u.z) < 1) {
        return Vec3f(-1, 1, 1);
    }
    return Vec3f(1.f - (u.x + u.y) / (float)u.z, u.x / (float)u.z, u.y / (float)u.z);
}


//质心坐标计算
Vec3f barycentric(Vec3f A,Vec3f B,Vec3f C,Vec3f P){
    Vec3f s[2];
    //计算[AB,AC,PA]的x和y分量
    for (int i = 2; i++;)
    {
        s[i][0] = C[i] - A[i];
        s[i][1] = B[i] - A[i];
        s[i][2] = A[i] - P[i];
    }
    // [u, v, 1]和[AB, AC, PA]对应的x和y向量都垂直，所以叉乘
    Vec3f u = cross(s[0], s[1]);
    //三点共线时，会导致u[2]为0，此时返回(-1,1,1)
    if (std::abs(u[2]) > 1e-2)
        //若1-u-v，u，v全为大于0的数，表示点在三角形内部
        return Vec3f(1.f - (u.x + u.y) / u.z, u.y / u.z, u.x / u.z);
    return Vec3f(-1, 1, 1);
}
void drawSingleTriangle() {
    // 图片的宽高
    int width = 200;
    int height = 200;

    TGAImage frame(width, height, TGAImage::RGB);
    Vec2i pts[3] = { Vec2i(10, 10), Vec2i(150, 30), Vec2i(70, 160) };

    Vec2i P;
    // 遍历图片中的所有像素
    for (P.x = 0; P.x <= width - 1; P.x++) {
        for (P.y = 0; P.y <= height - 1; P.y++) {
            Vec3f bc_screen = barycentric2D(pts, P);

            // bc_screen 某个分量小于 0 则表示此点在三角形外（认为边也是三角形的一部分）
            if (bc_screen.x < 0 || bc_screen.y < 0 || bc_screen.z < 0) {
                continue;
            }

            frame.set(P.x, P.y, white);
        }
    }

    frame.flip_vertically();
    frame.write_tga_file("day03_cross_product_triangle.tga");
}

void Triangle(Vec3f *pts, float *zbuffer,TGAImage &image,TGAColor color)
{
    Vec2f bboxmin(std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
    Vec2f bboxmax(std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
    Vec2f clamp(image.get_width() - 1, image.get_height() - 1);
    //确定包围框
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 2; j++)
        {
            bboxmin[j] = std::max(0.f,std::min(bboxmin[j],pts[i][j]));

            bboxmax[j] = std::min(clamp[j],std::max(bboxmax[j],pts[i][j]));
        }
    }

    Vec3f P;
    for ( P.x = bboxmin.x; P.x < bboxmax.x; P.x++)
    {
        for (P.y = bboxmin.y; P.y < bboxmax.y; P.y++)
        {
            Vec3f bc_screen = barycentric(pts[0], pts[1], pts[2], P);
            //通过质心坐标计算是否在三角形内
            if (bc_screen.x < 0 || bc_screen.y < 0 || bc_screen.z < 0) continue;
            P.z = 0;
            for (int i = 0; i < 3; i++) P.z += pts[i][2] * bc_screen[i];
            if (zbuffer[int(P.x + P.y * width)] < P.z) {
                zbuffer[int(P.x + P.y * width)] = P.z;
                image.set(P.x, P.y, color);
            }
        }
    }
}

Vec3f world2screen(Vec3f v) {
    return Vec3f(int((v.x + 1.) * width / 2. + .5), int((v.y + 1.) * height / 2. + .5), v.z);
}

int main(int argc, char** argv) {
    if (2 == argc) {
        model = new Model(argv[1]);
    }
    else {
        model = new Model("obj/african_head.obj");
    }

    float* zbuffer = new float[width * height];
    for (int i = width * height; i--; zbuffer[i] = -std::numeric_limits<float>::max());

    TGAImage image(width, height, TGAImage::RGB);
    for (int i = 0; i < model->nfaces(); i++) {
        std::vector<int> face = model->face(i);
        Vec3f pts[3];
        for (int i = 0; i < 3; i++) pts[i] = world2screen(model->vert(face[i]));
        Triangle(pts, zbuffer, image, TGAColor(rand() % 255, rand() % 255, rand() % 255, 255));
    }

    image.flip_vertically(); // i want to have the origin at the left bottom corner of the image
    image.write_tga_file("output.tga");
    delete model;
    return 0;
    //drawSingleTriangle();

}

