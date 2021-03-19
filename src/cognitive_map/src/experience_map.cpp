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

#include "experience_map.h"
#include "utils.h"

#include <queue>
#include <float.h>
#include <iostream>

using namespace std;

ExperienceMap::ExperienceMap(ptree settings) {
    get_setting_from_ptree(EXP_CORRECTION, settings, "exp_correction", 0.5);
    get_setting_from_ptree(EXP_LOOPS, settings, "exp_loops", 10);
    get_setting_from_ptree(EXP_INITIAL_EM_DEG, settings, "exp_initial_em_deg", 90.0);

    experiences.reserve(10000);
    links.reserve(10000);

    current_exp_id = 0;
    prev_exp_id = 0;

    accum_delta_facing = EXP_INITIAL_EM_DEG * M_PI / 180;
    accum_delta_x = 0;
    accum_delta_y = 0;
    accum_delta_time_s = 0;
}

ExperienceMap::~ExperienceMap() {
    links.clear();
    experiences.clear();
}

// create a new experience for a given position 
int ExperienceMap::on_create_experience(unsigned int exp_id) {

    experiences.resize(experiences.size() + 1);
    Experience *new_exp = &(*(experiences.end() - 1));

    if (experiences.size() == 0) {
        new_exp->x_m = 0;
        new_exp->y_m = 0;
        new_exp->th_rad = 0;
    } else {
        new_exp->x_m = experiences[current_exp_id].x_m + accum_delta_x;
        new_exp->y_m = experiences[current_exp_id].y_m + accum_delta_y;
        new_exp->th_rad = clip_rad_180(accum_delta_facing);
    }
    new_exp->id = experiences.size() - 1;

    if (experiences.size() != 1)
        on_create_link(get_current_id(), experiences.size() - 1, 0);

    return experiences.size() - 1;
}

// update the current position of the experience map
// since the last experience
void ExperienceMap::on_odo(double vtrans, double vrot, double time_diff_s) {
    vtrans = vtrans * time_diff_s;
    vrot = vrot * time_diff_s;
    accum_delta_facing = clip_rad_180(accum_delta_facing + vrot);
    accum_delta_x = accum_delta_x + vtrans * cos(accum_delta_facing);
    accum_delta_y = accum_delta_y + vtrans * sin(accum_delta_facing);
    accum_delta_time_s += time_diff_s;
}

// iterate the experience map. Perform a graph relaxing algorithm to allow
// the map to partially converge.
bool ExperienceMap::iterate() {
    int i;
    unsigned int link_id;
    unsigned int exp_id;
    Experience *link_from, *link_to;
    Link *link;
    double lx, ly, df;

    for (i = 0; i < EXP_LOOPS; i++) {
        for (exp_id = 0; exp_id < experiences.size(); exp_id++) {
            link_from = &experiences[exp_id];

            for (link_id = 0; link_id < link_from->links_from.size(); link_id++) {
                //%             //% experience 0 has a link to experience 1
                link = &links[link_from->links_from[link_id]];
                link_to = &experiences[link->exp_to_id];

                //%             //% work out where e0 thinks e1 (x,y) should be based on the stored
                //%             //% link information
                lx = link_from->x_m + link->d * cos(link_from->th_rad + link->heading_rad);
                ly = link_from->y_m + link->d * sin(link_from->th_rad + link->heading_rad);

                //%             //% correct e0 and e1 (x,y) by equal but opposite amounts
                //%             //% a 0.5 correction parameter means that e0 and e1 will be fully
                //%             //% corrected based on e0's link information
                link_from->x_m += (link_to->x_m - lx) * EXP_CORRECTION;
                link_from->y_m += (link_to->y_m - ly) * EXP_CORRECTION;
                link_to->x_m -= (link_to->x_m - lx) * EXP_CORRECTION;
                link_to->y_m -= (link_to->y_m - ly) * EXP_CORRECTION;

                //%             //% determine the angle between where e0 thinks e1's facing
                //%             //% should be based on the link information
                df = get_signed_delta_rad(link_from->th_rad + link->facing_rad, link_to->th_rad);

                //%             //% correct e0 and e1 facing by equal but opposite amounts
                //%             //% a 0.5 correction parameter means that e0 and e1 will be fully
                //%             //% corrected based on e0's link information
                link_from->th_rad = clip_rad_180(link_from->th_rad + df * EXP_CORRECTION);
                link_to->th_rad = clip_rad_180(link_to->th_rad - df * EXP_CORRECTION);
            }
        }
    }

    return true;
}

// create a link between two experiences
bool ExperienceMap::on_create_link(int exp_id_from, int exp_id_to, double rel_rad) {
    Experience *current_exp = &experiences[exp_id_from];

    // check if the link already exists
    for (unsigned int i = 0; i < experiences[exp_id_from].links_from.size(); i++) {
        if (links[experiences[current_exp_id].links_from[i]].exp_to_id == exp_id_to)
            return false;
    }

    for (unsigned int i = 0; i < experiences[exp_id_to].links_from.size(); i++) {
        if (links[experiences[exp_id_to].links_from[i]].exp_to_id == exp_id_from)
            return false;
    }

    links.resize(links.size() + 1);
    Link *new_link = &(*(links.end() - 1));

    new_link->exp_to_id = exp_id_to;
    new_link->exp_from_id = exp_id_from;
    new_link->d = sqrt(accum_delta_x * accum_delta_x + accum_delta_y * accum_delta_y);
    new_link->heading_rad = get_signed_delta_rad(current_exp->th_rad, atan2(accum_delta_y, accum_delta_x));
    new_link->facing_rad = get_signed_delta_rad(current_exp->th_rad, clip_rad_180(accum_delta_facing + rel_rad));

    // add this link to the 'to exp' so we can go backwards through the em
    experiences[exp_id_from].links_from.push_back(links.size() - 1);
    experiences[exp_id_to].links_to.push_back(links.size() - 1);

    return true;
}

// change the current experience
int ExperienceMap::on_set_experience(int new_exp_id, double rel_rad) {
    if (new_exp_id > experiences.size() - 1)
        return 0;

    if (new_exp_id == current_exp_id) {
        return 1;
    }

    prev_exp_id = current_exp_id;
    current_exp_id = new_exp_id;
    accum_delta_x = 0;
    accum_delta_y = 0;
    accum_delta_facing = clip_rad_180(experiences[current_exp_id].th_rad + rel_rad);

    return 1;
}
