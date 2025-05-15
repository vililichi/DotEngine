#include "./interfaces.hpp"
#include <array>
#include <iostream>
#include <omp.h>
#include <algorithm>
#pragma once

constexpr size_t COLLISION_SORTER_MAX_DEPT = 16;
constexpr size_t COLLISION_SORTER_MINIMUM_BODY = 96;

struct CollisiontQuadSortingResult
{
    public:
    std::vector<size_t> zones[4];
    std::vector<size_t> zone_hybrid;
    std::vector<std::array<bool, 4>> zone_hybrid_valid;

    constexpr static size_t  zone_xpos_ypos_id = 0;
    constexpr static size_t  zone_xpos_yneg_id = 1;
    constexpr static size_t  zone_xneg_ypos_id = 2;
    constexpr static size_t  zone_xneg_yneg_id = 3;
};

CollisiontQuadSortingResult collision_quad_sort(const std::vector<std::shared_ptr<DotBodyInterface>>& body_ptrs, const std::vector<size_t>& body_ids)
{
    // Reserve place for output
    const size_t nbr_body = body_ids.size();
    CollisiontQuadSortingResult result;
    result.zones[0].reserve(nbr_body);
    result.zones[1].reserve(nbr_body);
    result.zones[2].reserve(nbr_body);
    result.zones[3].reserve(nbr_body);
    result.zone_hybrid.reserve(nbr_body);
    result.zone_hybrid_valid.reserve(nbr_body);

    // finding pivot point
    Float2d pivot_point;
    for(size_t i = 0 ; i < nbr_body; i++)
    {
        pivot_point += body_ptrs[body_ids[i]]->get_position();
    }
    pivot_point /= static_cast<float>(nbr_body);
    const float x_pivot  = pivot_point.x();
    const float y_pivot  = pivot_point.y();

    // Sort point
    for(size_t i = 0 ; i < nbr_body; i++)
    {
        const Float2d position = body_ptrs[body_ids[i]]->get_position();
        const float size = body_ptrs[body_ids[i]]->get_size();
        const float x = position.x();
        const float y = position.y();

        const float min_x = x-size;
        const float min_y = y-size;

        const float max_x = x+size;
        const float max_y = y+size;

        std::array<bool, 4> zone_hybrid_valid {{false,false,false,false}};
        uint8_t nbr_zone = 0;
        size_t id_to_add = 0;

        if(min_x < x_pivot && min_y < y_pivot) 
        {
            nbr_zone += 1;
            zone_hybrid_valid[CollisiontQuadSortingResult::zone_xneg_yneg_id] = true;
            id_to_add = CollisiontQuadSortingResult::zone_xneg_yneg_id;
        }
        if(min_x < x_pivot && max_y > y_pivot) 
        {
            nbr_zone += 1;
            zone_hybrid_valid[CollisiontQuadSortingResult::zone_xneg_ypos_id] = true;
            id_to_add = CollisiontQuadSortingResult::zone_xneg_ypos_id;
        }
        if(max_x > x_pivot && min_y < y_pivot) 
        {
            nbr_zone += 1;
            zone_hybrid_valid[CollisiontQuadSortingResult::zone_xpos_yneg_id] = true;
            id_to_add = CollisiontQuadSortingResult::zone_xpos_yneg_id;
        }
        if(max_x > x_pivot && max_y > y_pivot) 
        {
            nbr_zone += 1;
            zone_hybrid_valid[CollisiontQuadSortingResult::zone_xpos_ypos_id] = true;
            id_to_add = CollisiontQuadSortingResult::zone_xpos_ypos_id;
        }

        #pragma omp critical
        {
            if(nbr_zone == 1) result.zones[id_to_add].push_back(body_ids[i]);
            else{
                result.zone_hybrid.push_back(body_ids[i]);
                result.zone_hybrid_valid.push_back(zone_hybrid_valid);
            }
        }
    }

    return result;

}

struct CollisionPoolResult
{
    size_t master_id;
    std::vector<size_t> slave_ids;
};

std::vector<CollisionPoolResult> generate_collision_pool(const std::vector<std::shared_ptr<DotBodyInterface>>& body_ptrs, std::vector<size_t> body_ids = std::vector<size_t>(), const size_t depth = 0 )
{

    if(body_ids.size() == 0){
        body_ids.resize(body_ptrs.size());
        for( size_t i = 0 ; i < body_ids.size(); i++)body_ids[i] = i;
    }

    std::vector<CollisionPoolResult> out;
    out.reserve(body_ptrs.size());

    // Early exit
    if(body_ids.size() < COLLISION_SORTER_MINIMUM_BODY || depth >= COLLISION_SORTER_MAX_DEPT){
        for( size_t i = 0 ; i < body_ids.size(); i++ )
        {
            const size_t body_id_offset = i+1;
            const size_t result_size = body_ids.size()-body_id_offset;
            if(result_size > 0)
            {
                CollisionPoolResult result;
                result.master_id = body_ids[i];

                result.slave_ids.resize(result_size);
                std::memcpy(result.slave_ids.data(), body_ids.data() + body_id_offset, result_size*sizeof(size_t));

                out.push_back(result);
            }
        }
        return out;
    }

    CollisiontQuadSortingResult quad_sorting_result = collision_quad_sort(body_ptrs, body_ids);
    const size_t zones_sizes[4] = {quad_sorting_result.zones[0].size(), quad_sorting_result.zones[1].size(), quad_sorting_result.zones[2].size(), quad_sorting_result.zones[3].size()};
    const size_t hybrid_size = quad_sorting_result.zone_hybrid.size();

    for( size_t i = 0 ; i < hybrid_size; i++ )
    {
        CollisionPoolResult result;
        result.master_id = quad_sorting_result.zone_hybrid[i];

        size_t size_to_reserve = (hybrid_size-i);
        for( uint8_t k = 0; k < 4; k++)
        {
            if(quad_sorting_result.zone_hybrid_valid[i][k])
            {
                size_to_reserve += zones_sizes[k];
            }
        }

        result.slave_ids.resize(size_to_reserve);
        size_t real_size = 0;

        for( size_t j = (i+1) ; j < hybrid_size; j++ )
        {
            if(
                (quad_sorting_result.zone_hybrid_valid[i][0] && quad_sorting_result.zone_hybrid_valid[j][0]) ||
                (quad_sorting_result.zone_hybrid_valid[i][1] && quad_sorting_result.zone_hybrid_valid[j][1]) ||
                (quad_sorting_result.zone_hybrid_valid[i][2] && quad_sorting_result.zone_hybrid_valid[j][2]) ||
                (quad_sorting_result.zone_hybrid_valid[i][3] && quad_sorting_result.zone_hybrid_valid[j][3])
            )
            {
                result.slave_ids[real_size] = quad_sorting_result.zone_hybrid[j];
                real_size += 1;
            }
        }

        for( uint8_t k = 0; k < 4; k++)
        {
            if(quad_sorting_result.zone_hybrid_valid[i][k])
            {
                //for( size_t j = 0 ; j < zones_sizes[k]; j++ ) result.slave_ids[real_size+j] = quad_sorting_result.zones[k][j];
                std::memcpy(result.slave_ids.data()+real_size, quad_sorting_result.zones[k].data(), zones_sizes[k]*sizeof(size_t));
                real_size += zones_sizes[k];
            }
        }

        result.slave_ids.resize(real_size);

        out.push_back(result);
    }

    for( uint8_t k = 0; k < 4; k++)
    {
        std::vector<CollisionPoolResult> recursive_result = generate_collision_pool(body_ptrs, quad_sorting_result.zones[k], depth + 1);
        const size_t out_size = out.size();
        const size_t result_size = recursive_result.size();
        out.resize(out_size + result_size);

        for( size_t i = 0 ; i < recursive_result.size();  i++)
        {
            out[i + out_size] = std::move(recursive_result[i]);
        }
    }

    return out;
}