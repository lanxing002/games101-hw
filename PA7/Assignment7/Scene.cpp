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
    // position in main scene
    auto pos = intersect(ray);
    if (!pos.happened) return Vector3f(.0, .0, .0);

    Vector3f c =  pos.happened ? (pos.normal + 1) * 0.5 : Vector3f(.0, .0, .0);
    
    // direct light
    {
        float pdf = 0.0;
        Vector3f l{.0, .0, .0};
        Intersection lightInter;
        sampleLight(lightInter, pdf);
        Vector3f ws = (lightInter.coords - pos.coords);
        Ray lightRay{ pos.coords, ws};
        auto pToLight = intersect(lightRay);
        if (!pToLight.happened || pToLight.obj->hasEmit() ){
            l += lightInter.emit * dotProduct(ws, pos.normal) * dotProduct(ws, lightInter.normal);
            l = l / std::max((lightInter.coords - pos.coords).norm(), 1.0f) / pdf;
            l = l * pos.m->eval(ray.direction, ws, pos.normal);
        }

        return l;
        //if(lightObj)

        //// blocked
        //if (lightObj != nullptr && lightObj == pToLight.obj){
        //}
    }



    // TO DO Implement Path Tracing Algorithm here
    return c;
}