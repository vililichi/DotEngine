#include "./interfaces.hpp"
#include <array>
#include <iostream>
#include <algorithm>
#pragma once

constexpr size_t COLLISION_SORTER_MAX_DEPT = 8;
constexpr size_t COLLISION_SORTER_MINIMUM_BODY = 64;

constexpr size_t ZONES_RESULT_MEMORY_SEGMENT_NBR = 500;
class ZonesResultMemory{
    private:
        static std::array<std::vector<size_t>,4> zones_result[ZONES_RESULT_MEMORY_SEGMENT_NBR];
        static size_t counter;
    public:
        static bool available()
        {
            return counter < ZONES_RESULT_MEMORY_SEGMENT_NBR;
        }
        static std::array<std::vector<size_t>,4>& get()
        {
            counter += 1;
            return zones_result[counter - 1];
        }
        static void reset() {
            counter = 0;
        }
};
std::array<std::vector<size_t>,4> ZonesResultMemory::zones_result[ZONES_RESULT_MEMORY_SEGMENT_NBR];
size_t ZonesResultMemory::counter = 0;


void collision_quad_sort(
    const std::vector<std::shared_ptr<DotBodyInterface>>& body_ptrs, 
    const std::vector<size_t>& body_ids, 
    std::vector<size_t>& zone_hybrid_result,
    std::vector<std::array<bool, 4>>& zone_hybrid_valid_result,
    std::array<std::vector<size_t>,4>& zones_result
)
{

    // Reserve place for output
    const size_t nbr_body = body_ids.size();
    
    zones_result[0].resize(nbr_body);
    zones_result[1].resize(nbr_body);
    zones_result[2].resize(nbr_body);
    zones_result[3].resize(nbr_body);
    zone_hybrid_result.resize(nbr_body);
    zone_hybrid_valid_result.resize(nbr_body);
    size_t nbr_body_in_zone[4] = {0,0,0,0};
    size_t nbr_body_hybrid_zone = 0;

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
            zone_hybrid_valid[0] = true;
            id_to_add = 0;
        }
        if(min_x < x_pivot && max_y > y_pivot) 
        {
            nbr_zone += 1;
            zone_hybrid_valid[1] = true;
            id_to_add = 1;
        }
        if(max_x > x_pivot && min_y < y_pivot) 
        {
            nbr_zone += 1;
            zone_hybrid_valid[2] = true;
            id_to_add = 2;
        }
        if(max_x > x_pivot && max_y > y_pivot) 
        {
            nbr_zone += 1;
            zone_hybrid_valid[3] = true;
            id_to_add = 3;
        }

        if(nbr_zone == 1) {
            zones_result[id_to_add][nbr_body_in_zone[id_to_add]] = body_ids[i];
            nbr_body_in_zone[id_to_add] += 1;
        }
        else{
            zone_hybrid_result[nbr_body_hybrid_zone]=body_ids[i];
            zone_hybrid_valid_result[nbr_body_hybrid_zone] = zone_hybrid_valid;
            nbr_body_hybrid_zone += 1;
        }
    }

    zones_result[0].resize(nbr_body_in_zone[0]);
    zones_result[1].resize(nbr_body_in_zone[1]);
    zones_result[2].resize(nbr_body_in_zone[2]);
    zones_result[3].resize(nbr_body_in_zone[3]);
    zone_hybrid_result.resize(nbr_body_hybrid_zone);
    zone_hybrid_valid_result.resize(nbr_body_hybrid_zone);

    return;

}

size_t generate_collision_pool_imp(
    const std::vector<std::shared_ptr<DotBodyInterface>>& body_ptrs, 
    const std::vector<size_t>& body_ids, 
    std::vector<std::vector<size_t>>& out, 
    const size_t initial_out_size, 
    const size_t depth,
    std::vector<size_t>& zone_hybrid_result,
    std::vector<std::array<bool, 4>>& zone_hybrid_valid_result
)
{
    const size_t nbr_body = body_ids.size();
    size_t size_out = initial_out_size;

    // Early exit
    if(nbr_body < COLLISION_SORTER_MINIMUM_BODY || depth >= COLLISION_SORTER_MAX_DEPT){
        for( size_t i = 0 ; i < nbr_body; i++ )
        {
            const size_t body_id_offset = i+1;
            const size_t result_size = nbr_body-body_id_offset;
            if(result_size > 0)
            {
                out[size_out].resize(result_size+1);
                out[size_out][0] = body_ids[i];
                std::memcpy(out[size_out].data()+1, body_ids.data() + body_id_offset, result_size*sizeof(size_t));
                size_out += 1;
            }
        }
        return size_out;
    }

    std::array<std::vector<size_t>,4> backup;
    std::array<std::vector<size_t>,4> zones_result = ZonesResultMemory::available() ? ZonesResultMemory::get() : backup;
    collision_quad_sort(body_ptrs, body_ids, zone_hybrid_result, zone_hybrid_valid_result, zones_result);
    const size_t zones_sizes[4] = {zones_result[0].size(), zones_result[1].size(), zones_result[2].size(), zones_result[3].size()};
    const size_t hybrid_size = zone_hybrid_result.size();

    for( size_t i = 0 ; i < hybrid_size; i++ )
    {
        size_t size_to_reserve = (hybrid_size-i);
        for( uint8_t k = 0; k < 4; k++)
        {
            if(zone_hybrid_valid_result[i][k])
            {
                size_to_reserve += zones_sizes[k];
            }
        }

        out[size_out].resize(size_to_reserve+1);
        out[size_out][0] = zone_hybrid_result[i];
        size_t real_size = 1;

        for( size_t j = (i+1) ; j < hybrid_size; j++ )
        {
            if(
                (zone_hybrid_valid_result[i][0] && zone_hybrid_valid_result[j][0]) ||
                (zone_hybrid_valid_result[i][1] && zone_hybrid_valid_result[j][1]) ||
                (zone_hybrid_valid_result[i][2] && zone_hybrid_valid_result[j][2]) ||
                (zone_hybrid_valid_result[i][3] && zone_hybrid_valid_result[j][3])
            )
            {
                out[size_out][real_size] = zone_hybrid_result[j];
                real_size += 1;
            }
        }

        for( uint8_t k = 0; k < 4; k++)
        {
            if(zone_hybrid_valid_result[i][k])
            {
                std::memcpy(out[size_out].data()+real_size, zones_result[k].data(), zones_sizes[k]*sizeof(size_t));
                real_size += zones_sizes[k];
            }
        }

        out[size_out].resize(real_size);
        size_out += 1;
    }

    for( uint8_t k = 0; k < 4; k++)
    {
        size_out = generate_collision_pool_imp(body_ptrs, zones_result[k], out, size_out, depth + 1, zone_hybrid_result, zone_hybrid_valid_result);
    }

    return size_out;
}

void generate_collision_pool(const std::vector<std::shared_ptr<DotBodyInterface>>& body_ptrs, std::vector<std::vector<size_t>>& out_buffer)
{
    ZonesResultMemory::reset();

    const size_t nbr_body = body_ptrs.size();

    std::vector<size_t> body_ids;
    body_ids.resize(nbr_body);
    for( size_t i = 0 ; i < body_ids.size(); i++)body_ids[i] = i;

    out_buffer.resize(nbr_body);

    std::vector<size_t> zone_hybrid_result;
    std::vector<std::array<bool, 4>> zone_hybrid_valid_result;

    const size_t out_size = generate_collision_pool_imp(body_ptrs, body_ids, out_buffer, 0, 0, zone_hybrid_result, zone_hybrid_valid_result);

    out_buffer.resize(out_size);

}