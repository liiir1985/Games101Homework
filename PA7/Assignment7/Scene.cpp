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
    auto inter = intersect(ray);
    if(!inter.happened)
        return Vector3f(0,0,0);
    auto p = inter.coords;
    float pdf_light;
    Intersection lightInter;
    sampleLight(lightInter, pdf_light);
    auto emit = lightInter.emit;
    auto x = lightInter.coords;
    auto w0 = -ray.direction;
    auto ws = normalize(x - p);
    Ray lightRay(inter.coords, ws);
    auto tmpInter = intersect(lightRay);
    Vector3f L_dir(0,0,0);
    if(!tmpInter.happened || (tmpInter.coords - lightInter.coords).norm() < 0.1f)
    {
        auto distance = (x - p).norm();
        auto eval = inter.m->eval(w0, ws, inter.normal);
        auto SdN = clamp(0, 1, dotProduct(ws, inter.normal) * dotProduct(-ws, lightInter.normal));
        auto norm = (distance * distance) * pdf_light;
        L_dir = emit * eval * SdN / (norm + EPSILON);
    }

    Vector3f L_ind(0, 0, 0);
    if(get_random_float() < RussianRoulette)
    {
        Vector3f wi = inter.m->sample(w0, inter.normal);
        Ray nextRay(p, wi);
        auto nextInter = intersect(nextRay);
        if(nextInter.happened && !nextInter.m->hasEmission())
        {
            L_ind = castRay(nextRay, depth + 1) * inter.m->eval(w0, wi, inter.normal) * dotProduct(wi, inter.normal) / (inter.m->pdf(w0, wi, inter.normal) * RussianRoulette + EPSILON);
        }
    }
    return L_dir + L_ind;
}