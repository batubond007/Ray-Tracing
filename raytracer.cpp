#include <iostream>
#include "parser.h"
#include "ppm.h"
#include <thread>

typedef unsigned char RGB[3];

parser::Ray GenerateRay(parser::Vec3f pixel_start_pos, int i, int j, float coeff_su, float coeff_sv);

parser::Vec3f CalculatePixelStartPointSphere();

parser::Intersection IntersectRay(parser::Ray ray);

parser::Intersection GetIntersectionPointSphere(parser::Ray ray, parser::Sphere sphere);

parser::Intersection GetIntersectionPointTriangle(parser::Ray ray, parser::Triangle triangle);

parser::Intersection GetIntersectionPointMesh(parser::Ray ray, parser::Mesh mesh, int meshID);

std::vector<std::vector<parser::Vec3f>> CalculateNormals();

parser::Vec3f AmbientColor(parser::Intersection intersection, parser::Material material);

parser::Vec3f DiffuseAndSpecularColor(parser::Intersection &intersection, parser::Material material, parser::Ray ray);

parser::Vec3f
CalculateColor(parser::Vec3f pixel_start_point, int i, int j, float coeff_su, float coeff_sv, int rec_depth,
               parser::Intersection _intersection);

parser::Ray CalculateRay(parser::Intersection intersection);

void ColorThread(parser::Vec3f pixel_start_point, int j, int end_j, float coeff_su, float coeff_sv, int depth, unsigned char *string);

parser::Camera camera;
parser::Scene scene;
std::vector<std::vector<parser::Vec3f>> MeshNormals;

int main(int argc, char* argv[])
{
    // Sample usage for reading an XML scene file
    scene.loadFromXml(argv[1]);

    for (int cam_index = 0; cam_index < scene.cameras.size(); ++cam_index) {
        camera = scene.cameras[cam_index];
        int width = camera.image_width;
        int height = camera.image_height;

        unsigned char *image = new unsigned char[width * height * 3];

        //Precalculations
        camera.u = camera.gaze.CrossProduct(camera.up);
        parser::Vec3f pixel_start_point = CalculatePixelStartPointSphere();
        float coeff_su = (camera.near_plane.y - camera.near_plane.x) / (float) width;
        float coeff_sv = (camera.near_plane.w - camera.near_plane.z) / (float) height;
        MeshNormals = CalculateNormals();

        int process_count = height / 8;
        std::vector<std::thread> threads;

        for (int j = 0; j < height; j+=process_count) {
            threads.push_back(std::thread(ColorThread, std::ref(pixel_start_point), j, j + process_count
                    , coeff_su, coeff_sv, scene.max_recursion_depth, image));
            //ColorThread(pixel_start_point, j, j + process_count, coeff_su, coeff_sv, scene.max_recursion_depth, image);
            if (j + process_count > height){
                //ColorThread(pixel_start_point, j, height, coeff_su, coeff_sv, scene.max_recursion_depth, image);
                threads.push_back(std::thread(ColorThread, std::ref(pixel_start_point), j, height, coeff_su, coeff_sv, scene.max_recursion_depth, image));
            }
        }

        for (int x = 0; x < threads.size(); ++x) {
            threads[x].join();
        }

        write_ppm(camera.image_name.c_str(), image, width, height);
    }
}

void ColorThread(parser::Vec3f pixel_start_point, int j, int end_j, float coeff_su, float coeff_sv, int depth, unsigned char *image) {
    for (j; j < end_j; ++j) {
        for (int i = 0; i < camera.image_width; i++) {
            parser::Intersection intersection;
            parser::Vec3f color = CalculateColor(pixel_start_point, i, j, coeff_su, coeff_sv,
                                                 scene.max_recursion_depth, intersection).Clamp();

            image[(j * camera.image_width + i) * 3 + 0] = color.x;
            image[(j * camera.image_width + i) * 3 + 1] = color.y;
            image[(j * camera.image_width + i) * 3 + 2] = color.z;
        }
    }
}

parser::Vec3f CalculateColor(parser::Vec3f pixel_start_point, int i, int j, float coeff_su, float coeff_sv, int rec_depth,
               parser::Intersection _intersection) {
    parser::Vec3f color;

    if (rec_depth == -1){
        color = {0,0,0};
        return color;
    }
    parser::Ray ray;
    if (rec_depth == scene.max_recursion_depth)
        ray = GenerateRay(pixel_start_point, i, j,coeff_su, coeff_sv);
    else
        ray = CalculateRay(_intersection);

    parser::Intersection intersection = IntersectRay(ray);
    parser::Material material;

    if (intersection.isSphere){
        material = scene.materials[intersection.sphere.material_id-1];
        color = AmbientColor(intersection, material);
        color = color + DiffuseAndSpecularColor(intersection, material, ray);
    }
    else if (intersection.isTriangle){
        material = scene.materials[intersection.triangle.material_id-1];
        color = AmbientColor(intersection, material);
        color = color + DiffuseAndSpecularColor(intersection, material, ray);
    }
    else{
        if (rec_depth == scene.max_recursion_depth){
            color.x = scene.background_color.x;
            color.y = scene.background_color.y;
            color.z = scene.background_color.z;
            return color;
        }
        return parser::Vec3f{0,0,0};
    }

    if(material.is_mirror){
        rec_depth--;
        return color + CalculateColor(pixel_start_point, i, j, coeff_su, coeff_sv, rec_depth, intersection) * material.mirror;
    }
    else
        return color;

}

parser::Ray CalculateRay(parser::Intersection intersection) {
    parser::Ray result;
    result.e = intersection.point + (intersection.normal * scene.shadow_ray_epsilon);
    result.d = ((intersection.ray.d) - (intersection.normal * 2) * intersection.normal.DotProduct(intersection.ray.d)).Normalize();
    return result;
}

parser::Vec3f DiffuseAndSpecularColor(parser::Intersection &intersection, parser::Material material, parser::Ray ray) {
    parser::Vec3f result;
    result.x = 0;
    result.y = 0;
    result.z = 0;
    for (int i = 0; i < scene.point_lights.size(); ++i) {
        parser::PointLight pointLight = scene.point_lights[i];
        parser::Vec3f toLight = (pointLight.position - intersection.point).Normalize();

        parser::Vec3f normal;
        if (intersection.isSphere){
            normal = (intersection.point - scene.vertex_data[intersection.sphere.center_vertex_id-1]).Normalize();
        }
        else if (intersection.MeshID == -1 && intersection.isTriangle){
            parser::Vec3f a = scene.vertex_data[intersection.face.v0_id - 1];
            parser::Vec3f b = scene.vertex_data[intersection.face.v1_id - 1];
            parser::Vec3f c = scene.vertex_data[intersection.face.v2_id - 1];
            normal = ((c-b).CrossProduct((a - b))).Normalize();
        }
        else if(intersection.MeshID != -1 && intersection.isTriangle) {
            normal = MeshNormals[intersection.MeshID][intersection.FaceID];
        }

        intersection.normal = normal;

        // Check Shadow
        parser::Ray newRay;
        newRay.e = (intersection.point + (normal * scene.shadow_ray_epsilon));
        newRay.d = (pointLight.position - newRay.e).Normalize();

        parser::Intersection newIntersection = IntersectRay(newRay);
        if ((newIntersection.isSphere || newIntersection.isTriangle)
        && (intersection.point.Distance(pointLight.position) > intersection.point.Distance(newIntersection.point))
        )
            continue;


        //Diffuse
        float cosa = normal.DotProduct(toLight);
        float radius = pointLight.position.Distance(intersection.point);
        if (cosa < 0) cosa = 0;
        result = result + (pointLight.intensity * (material.diffuse * (cosa / powf(radius, 2))));

        //Specular
        parser::Vec3f h = (toLight - ray.d.Normalize()) * (1 / (toLight - ray.d.Normalize()).Length());
        cosa = h.DotProduct(normal);
        if (cosa < 0) cosa = 0;
        parser::Vec3f specular = material.specular * (powf(cosa, material.phong_exponent)) * (pointLight.intensity  * (1 / powf(radius, 2)));
        result = result + specular;
    }
    return result;
}

parser::Vec3f AmbientColor(parser::Intersection intersection, parser::Material material) {
    parser::Vec3f result;
    result.x = material.ambient.x * scene.ambient_light.x;
    result.y = material.ambient.y * scene.ambient_light.y;
    result.z = material.ambient.z * scene.ambient_light.z;
    return result;
}

std::vector<std::vector<parser::Vec3f>> CalculateNormals() {
    std::vector<std::vector<parser::Vec3f>> result;
    for (int i = 0; i < scene.meshes.size(); ++i) {
        std::vector<parser::Vec3f> normals;
        parser::Mesh mesh = scene.meshes[i];
        for (int j = 0; j < mesh.faces.size(); ++j) {
            parser::Face face = mesh.faces[j];
            parser::Vec3f a = scene.vertex_data[face.v0_id - 1];
            parser::Vec3f b = scene.vertex_data[face.v1_id - 1];
            parser::Vec3f c = scene.vertex_data[face.v2_id - 1];
            parser::Vec3f normal = (b-a).CrossProduct((c-a));
            normal = normal.Normalize();
            normals.push_back(normal);
        }
        result.push_back(normals);
    }
    return result;
}

parser::Intersection IntersectRay(parser::Ray ray) {

    parser::Intersection result;
    result.isSphere = false;
    result.isTriangle = false;
    result.t = INT16_MAX;

    // Check For Spheres
    for (int i = 0; i < scene.spheres.size(); ++i) {
        parser::Sphere sphere = scene.spheres[i];
        parser::Intersection intersection = GetIntersectionPointSphere(ray, sphere);
        if (intersection.t < result.t && intersection.t > 0) {
            result = intersection;
        }
    }
    // Check For Triangles
    for (int i = 0; i < scene.triangles.size(); ++i) {
        parser::Triangle triangle = scene.triangles[i];
        parser::Intersection intersection = GetIntersectionPointTriangle(ray, triangle);
        if (intersection.t < result.t && intersection.t > 0) {
            result = intersection;
        }
    }
    // Check For Meshes
    for (int i = 0; i < scene.meshes.size(); ++i) {
        parser::Mesh mesh = scene.meshes[i];
        parser::Intersection intersection = GetIntersectionPointMesh(ray, mesh, i);
        if (intersection.t < result.t && intersection.t > 0) {
            result = intersection;
        }
    }

    // Compute Intersection Variables
    result.ray = ray;


    return result;
}

parser::Intersection GetIntersectionPointMesh(parser::Ray ray, parser::Mesh mesh, int meshID) {
    parser::Intersection result;
    result.isSphere = false;
    result.isTriangle = false;
    result.MeshID = -1;
    result.t = INT16_MAX;

    for (int i = 0; i < mesh.faces.size(); ++i) {
        /*float cullingCheck = ray.d.DotProduct(MeshNormals[meshID][i]);
        if (cullingCheck > 0)
            continue;*/

        parser::Face face = mesh.faces[i];

        parser::Triangle triangle;
        triangle.indices = face;
        triangle.material_id = mesh.material_id;

        parser::Intersection intersection = GetIntersectionPointTriangle(ray, triangle);
        if (intersection.t < result.t && intersection.t > 0){
            result = intersection;
            result.MeshID = meshID;
            result.FaceID = i;
        }
    }
    return result;
}

parser::Intersection GetIntersectionPointTriangle(parser::Ray ray, parser::Triangle triangle) {
    parser::Vec3f a = scene.vertex_data[triangle.indices.v0_id - 1];
    parser::Vec3f b = scene.vertex_data[triangle.indices.v1_id - 1];
    parser::Vec3f c = scene.vertex_data[triangle.indices.v2_id - 1];

    float matrix[3][4] = {
            {a.x - b.x, a.x - c.x, ray.d.x, a.x - ray.e.x },
            {a.y - b.y, a.y - c.y, ray.d.y, a.y - ray.e.y },
            {a.z - b.z, a.z - c.z, ray.d.z, a.z - ray.e.z }
    };

    parser::Vec3f solution = parser::CramerSolver(matrix);

    parser::Intersection result;
    result.isSphere = false;
    result.isTriangle = false;
    result.MeshID = -1;
    result.t = INT16_MAX;

    if(solution.x + solution.y <= 1 && solution.x >= 0 && solution.y >= 0){
        result.isTriangle = true;
        result.triangle = triangle;
        result.face = triangle.indices;
        result.t = solution.z;
        result.point = ray.e + ray.d * result.t;
    }
    return result;
}

parser::Intersection GetIntersectionPointSphere(parser::Ray ray, parser::Sphere sphere) {

    // Formula from slide 23
    parser::Vec3f c = scene.vertex_data[sphere.center_vertex_id - 1];
    float t_b = 2 * ray.d.DotProduct(ray.e - c);
    float t_a = ray.d.DotProduct(ray.d);
    float t_c = (ray.e-c).DotProduct(ray.e-c) - pow(sphere.radius, 2);
    float t_root = pow(t_b, 2) - (4 * t_a * t_c);

    parser::Intersection result;

    if (t_root < 0){
        result.t = INT16_MAX;
    }
    else if (t_root == 0){
        result.t = -t_b / (2 * t_a);
        result.isSphere = true;
        result.sphere = sphere;
        result.point =  ray.e + ray.d * result.t;
    }
    else{
        result.t = (-t_b - sqrtf(t_root)) / (2 * t_a);
        result.isSphere = true;
        result.sphere = sphere;
        result.point = ray.e + ray.d * result.t;
    }
    return result;
}

parser::Vec3f CalculatePixelStartPointSphere() {
    parser::Vec3f m = camera.position + camera.gaze * camera.near_distance;
    parser::Vec3f q = m + camera.u * camera.near_plane.x + camera.up * camera.near_plane.w;
    return q;
}

/* Takes index of width as i and height as j, returns ray from camera to the pixel */
parser::Ray GenerateRay(parser::Vec3f pixel_start_pos, int i, int j, float coeff_su, float coeff_sv) {
    float su = (i + 0.5f) * coeff_su;
    float sv = (j + 0.5f) * coeff_sv;

    parser::Vec3f s = pixel_start_pos + camera.u * su - camera.up * sv;
    parser::Ray ray;
    ray.e = camera.position;
    ray.d = s - camera.position;

    return  ray;
}
