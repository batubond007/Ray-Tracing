#ifndef __HW1__PARSER__
#define __HW1__PARSER__

#include <string>
#include <vector>
#include <cmath>

namespace parser
{
    //Notice that all the structures are as simple as possible
    //so that you are not enforced to adopt any style or design.
    struct Vec3f
    {
        float x, y, z;

        float Distance(Vec3f vec) const{
            float _x = powf(x - vec.x, 2);
            float _y = powf(y - vec.y, 2);
            float _z = powf(z - vec.z, 2);
            return sqrtf(_x+ _y+ _z);
        }
        float Length() const{
            float _x = powf(x, 2);
            float _y = powf(y, 2);
            float _z = powf(z, 2);
            return sqrtf(_x+ _y+ _z);
        }
        Vec3f Normalize() const{
            Vec3f result;
            float length = Length();
            result.x = x / length;
            result.y = y / length;
            result.z = z / length;

            return result;
        }
        Vec3f operator +(Vec3f const &vec) const{
            Vec3f result;
            result.x = x + vec.x;
            result.y = y + vec.y;
            result.z = z + vec.z;
            return  result;
        }
        Vec3f operator -(Vec3f const &vec) const{
            Vec3f result;
            result.x = x - vec.x;
            result.y = y - vec.y;
            result.z = z - vec.z;
            return  result;
        }
        Vec3f operator *(int const &num) const{
            Vec3f result;
            result.x = x * num;
            result.y = y * num;
            result.z = z * num;
            return  result;
        }
        Vec3f operator *(float const &num) const{
            Vec3f result;
            result.x = x * num;
            result.y = y * num;
            result.z = z * num;
            return  result;
        }
        Vec3f operator *(Vec3f const &vec) const{
            Vec3f result;
            result.x = x * vec.x;
            result.y = y * vec.y;
            result.z = z * vec.z;
            return  result;
        }
        float DotProduct(Vec3f vec) const{
            return  (x * vec.x + y * vec.y + z * vec.z);
        }
        Vec3f CrossProduct(Vec3f vec) const{
            Vec3f result;
            result.x = y * vec.z - z * vec.y;
            result.y = z * vec.x - x * vec.z;
            result.z = x * vec.y - y * vec.x;
            return result;
        }
        Vec3f Clamp() const {
            Vec3f result;
            result.x = x > 255 ? 255 : x;
            result.y = y > 255 ? 255 : y;
            result.z = z > 255 ? 255 : z;
            return result;
        }
    };

    struct Vec3i
    {
        int x, y, z;
        float Distance(Vec3i vec){
            int _x = pow(x - vec.x, 2);
            int _y = pow(y - vec.y, 2);
            int _z = pow(z - vec.z, 2);
            return sqrtf(_x+ _y+ _z);
        }
        float Length() const{
            int _x = pow(x, 2);
            int _y = pow(y, 2);
            int _z = pow(z, 2);
            return sqrtf(_x+ _y+ _z);
        }
    };

    struct Vec4f
    {
        float x, y, z, w;
    };

    struct Camera
    {
        Vec3f position;
        Vec3f gaze;
        Vec3f up;
        Vec3f u;
        Vec4f near_plane;
        float near_distance;
        int image_width, image_height;
        std::string image_name;
    };

    struct PointLight
    {
        Vec3f position;
        Vec3f intensity;
    };

    struct Material
    {
        bool is_mirror;
        Vec3f ambient;
        Vec3f diffuse;
        Vec3f specular;
        Vec3f mirror;
        float phong_exponent;
    };

    struct Face
    {
        int v0_id;
        int v1_id;
        int v2_id;
    };

    struct Mesh
    {
        int material_id;
        std::vector<Face> faces;
    };

    struct Triangle
    {
        int material_id;
        Face indices;
    };

    struct Sphere
    {
        int material_id;
        int center_vertex_id;
        float radius;
    };

    struct Scene
    {
        //Data
        Vec3i background_color;
        float shadow_ray_epsilon;
        int max_recursion_depth;
        std::vector<Camera> cameras;
        Vec3f ambient_light;
        std::vector<PointLight> point_lights;
        std::vector<Material> materials;
        std::vector<Vec3f> vertex_data;
        std::vector<Mesh> meshes;
        std::vector<Triangle> triangles;
        std::vector<Sphere> spheres;

        //Functions
        void loadFromXml(const std::string &filepath);
    };

    struct Ray{
        Vec3f e;
        Vec3f d;
        float t;
    };

    struct Intersection{
        bool isSphere, isTriangle;
        Sphere sphere;
        Triangle triangle;
        Face face;
        float t;
        Vec3f point;
        int MeshID, FaceID;
        Ray ray;
        Vec3f normal;
    };

    // Returns a,b,t in order
    Vec3f CramerSolver(float matrix[3][4]);

    float Determinant(float matrix[3][3]);
}

#endif
