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
    auto ret = Vector3f(.0, .0, .0);

    auto pos = intersect(ray);
    if (!pos.happened) return ret;
    if (pos.m->hasEmission()) return pos.m->getEmission();

    auto wo = -ray.direction.normalized();
    auto n = pos.normal.normalized();

    // direct light
    {
        float pdf = 0.0;
        Intersection lightInter;
        sampleLight(lightInter, pdf);
        Vector3f ws = (lightInter.coords - pos.coords).normalized();
        //return (ws + 1.0) * 0.5 * 0.5;
        Ray lightRay{ pos.coords + ws * 0.0001, ws};
        auto pToLight = intersect(lightRay);
        if (pToLight.happened && pToLight.obj->hasEmit() ){
            auto l = lightInter.emit * dotProduct(ws, n) * dotProduct(-ws, lightInter.normal.normalized());
            auto v = lightInter.coords - pos.coords;
            l = l / std::max(dotProduct(v, v), 1.0f) / pdf;
            l = l * pos.m->eval(ray.direction, ws, pos.normal);
            ret = l;
        }
    }
    // indirect light
    {
        auto russianP = get_random_float();
        if (russianP < 0.7)
        {
            auto wi = pos.m->sample(wo, n).normalized();
            Ray indirRay{ pos.coords + wi * 0.0001, wi };
            auto pToScene = intersect(indirRay);
            // 直接光的贡献已经计算，只计算间接光
            if (pToScene.happened && !pToScene.obj->hasEmit()) {
                auto l = castRay(indirRay, depth + 1);
                //std::cout << "ll " << l << std::endl;
                l = l * pos.m->eval(wo, wi, n) * dotProduct(wi, n);
                l = l / pos.m->pdf(wi, wo, n) / 0.7;
                ret += l;
            }
        }
    }

    // TO DO Implement Path Tracing Algorithm here
    return ret;
}