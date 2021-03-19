/*
 * Reference: openRatSLAM GPLv3
 * David Ball (david.ball@qut.edu.au) (1), Scott Heath (scott.heath@uqconnect.edu.au) (2)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
/*
 * The Experience_Map class describes an undirected graph that is used
 * as the map component of RatSLAM. The Experience_Map class also handles
 * some aspects of goal based navigation.
 */

#ifndef _EXPERIENCE_MAP_H_
#define _EXPERIENCE_MAP_H_

#include "math.h"
#include <stdio.h>
#include <vector>
#include <deque>
#include <iostream>

#include <boost/property_tree/ini_parser.hpp>
#include <boost/serialization/access.hpp>
#include <boost/serialization/utility.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/deque.hpp>

using boost::property_tree::ptree;

/*
 * The Link structure describes a link
 * between two experiences.
 */

struct Link {
    double d;
    double heading_rad;
    double facing_rad;
    int exp_to_id;
    int exp_from_id;
};


/*
 * The Experience structure describes
 * a node in the Experience_Map.
 */
struct Experience {
    int id; // its own id
    double x_m, y_m, th_rad;

    std::vector<unsigned int> links_from; // links from this experience
    std::vector<unsigned int> links_to; // links to this experience
};


class ExperienceMap {
public:
    explicit ExperienceMap(ptree settings);

    ~ExperienceMap();

    // create a new experience for a given position
    int on_create_experience(unsigned int exp_id);

    bool on_create_link(int exp_id_from, int exp_id_to, double rel_rad);

    Experience *get_experience(int id) {
        return &experiences[id];
    }

    Link *get_link(int id) {
        return &links[id];
    }

    // update the current position of the experience map
    // since the last experience
    void on_odo(double vtrans, double vrot, double time_diff_s);

    // update the map by relaxing the graph
    bool iterate();

    // change the current experience
    int on_set_experience(int new_exp_id, double rel_rad);

    int get_num_experiences() {
        return experiences.size();
    }

    int get_num_links() {
        return links.size();
    }

    int get_current_id() const {
        return current_exp_id;
    }

private:
    ExperienceMap() = default;

    int EXP_LOOPS{};
    double EXP_CORRECTION{};
    double EXP_INITIAL_EM_DEG{};

    std::vector<Experience> experiences;
    std::vector<Link> links;

    int current_exp_id{}, prev_exp_id{};

    double accum_delta_facing{};
    double accum_delta_x{};
    double accum_delta_y{};
    double accum_delta_time_s{};
};

#endif // _EXPERIENCE_MAP_H_
