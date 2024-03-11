//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    Vector3f result = Vector3f(0);
    if (depth > this->maxDepth) {
        return Vector3f(0);
    }

    Intersection intersection = intersect(ray);
    if (!intersection.happened) {
        return Vector3f(0);
    }

    if (intersection.m->hasEmission()) {
        return intersection.m->getEmission();
    }

    Vector3f wo = normalize(-ray.direction);
    Vector3f p = intersection.coords;

    // Direct Lighting
    Vector3f L_dir = Vector3f(0);
    Intersection inter_dir;
    float pdf_light;
    sampleLight(inter_dir, pdf_light);

    Vector3f x = inter_dir.coords;
    Vector3f ws = normalize(x - p);
    Vector3f N = intersection.normal;
    Vector3f NN = inter_dir.normal;
    Vector3f emit = inter_dir.emit;

    Ray p_to_x(p, ws);
    Intersection shadow_inter = intersect(p_to_x);
    bool isBlocked = shadow_inter.happened && shadow_inter.distance < (x - p).norm();
    if (!isBlocked){
        float distance_square = (x - p).norm() * (x - p).norm();
        L_dir = emit * 
            intersection.m->eval(wo, ws, N) *
            dotProduct(ws, N) *
            dotProduct(-ws, NN) / (distance_square * pdf_light);
        // print out every element in the equation
        // std::cout << emit << " " << intersection.m->eval(wo, ws, N) << " " << dotProduct(ws, N) << " " << dotProduct(-ws, NN) << " " << distance_square << " " << pdf_light << std::endl;
        // std::cout << "L_dir: " << L_dir << std::endl;
    }

    // Indirect Lighting
    Vector3f L_indir = Vector3f(0);
    if(get_random_float() < RussianRoulette) {
        Vector3f wi = intersection.m->sample(wo, N);
        Ray p_to_wi(p, wi);
        float pdf_hemi = intersection.m->pdf(wi, wo, N);
        Vector3f fr = intersection.m->eval(wo, wi, N);
        Intersection inter_wi = intersect(p_to_wi);
        if(inter_wi.happened && !inter_wi.m->hasEmission()) {
            L_indir = fr * castRay(p_to_wi, depth + 1) * dotProduct(wi, N) / (pdf_hemi * RussianRoulette);
            // print out every element in the equation
            // std::cout << fr << " " << castRay(p_to_wi, depth + 1) << " " << dotProduct(wi, N) << " " << pdf_hemi << " " << RussianRoulette << std::endl;
            // std::cout << "L_indir: " << L_indir << std::endl;
        }
    }
    return L_dir + L_indir;

}