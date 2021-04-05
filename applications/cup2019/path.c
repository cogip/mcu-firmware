#include <stdio.h>

#include "path.h"
#include "app.h"

#include "utils.h"

/*
 * TODO:
 * - Add anti-blocking activation for each point,
 * - For each actions: keep a status if its done (to avoid multiple puck capture
 *    if delivery was bypassed for any reason)
 * - Absolute coordinate recalibration on games frames (?)
 */
static path_pose_t poses[] = {
    /* POSE_INITIAL */
    {
        .pos = {
            .x = 1500 - ROBOT_MARGIN,
            .y = 300 + ROBOT_MARGIN,
            .O = 180,
        },
        .allow_reverse = FALSE,
        .max_speed = MAX_SPEED,
        .act = NULL,
    },
    /* Game path */
    {                   /* Sortie zone de départ */
        .pos = {
            .x = 1275,  //1100,
            .y = 300 + ROBOT_MARGIN,
            .O = 90,
        },
        .allow_reverse = FALSE,
        .max_speed = NORMAL_SPEED,
        .act = NULL,
    },
    /* Point évitement zone du chaos x=800 y=1100 O=205+90*/
    {
        .pos = {
            .x = 1275,          //800,
            .y = 1300,          //1100,
            .O = 115,
        },
        .allow_reverse = FALSE,
        .max_speed = NORMAL_SPEED,
        .act = NULL,
    },
    {
        .pos = {
            .x = 600,           //800,
            .y = 1300,          //1100,
            .O = 90,
        },
        .allow_reverse = FALSE,
        .max_speed = NORMAL_SPEED,
        .act = NULL,
    },
    {
        /* Pré-Point */
        .pos = {
            .x = 600,
            .y = 1300,
            .O = 90,
        },
        .allow_reverse = FALSE,
        .max_speed = MAX_SPEED,
        .act = app_front_cup_take,
    },
    {
        /* Point récolte */
        /* TODO /!\ Régulation de vitesse seulement ? */
        .pos = {
            .x = 600,
            .y = 1361,
            .O = 90,
        },
        .allow_reverse = FALSE,
        .max_speed = LOW_SPEED,
        .act = NULL,
    },
    {
        /* Post-Point */
        .pos = {
            .x = 600,
            .y = 1300,
            .O = 90,
        },
        .allow_reverse = TRUE,
        .max_speed = MAX_SPEED,
        .act = app_front_cup_ramp,
    },
    /*
     * Récole 3 palets éloignés balance
     */
    {
        /* Pré-Point */
        .pos = {
            .x = 900,
            .y = 1300,
            .O = 270,
        },
        .allow_reverse = TRUE,
        .max_speed = NORMAL_SPEED,
        .act = app_back_cup_take,
    },
    {
        /* Point récolte */
        /* TODO /!\ Régulation de vitesse seulement ? */
        .pos = {
            .x = 900,
            .y = 1361,
            .O = 270,
        },
        .allow_reverse = TRUE,
        .max_speed = LOW_SPEED,
        .act = NULL,
    },
    {
        /* Post-Point */
        .pos = {
            .x = 900,
            .y = 1300,
            .O = 270,
        },
        .allow_reverse = TRUE,
        .max_speed = MAX_SPEED,
        .act = app_back_cup_ramp,
    },
    /*
     * Dépose dans la balance
     */
    {
        /* Point dépose rampe avant */
        .pos = {
            .x = 230,
            .y = 1310,
            .O = 180,        // 180: dépose cote gauche
        },
        .allow_reverse = FALSE,
        .max_speed = MAX_SPEED, //MAX_SPEED,
        .act = app_front_ramp_right_drop,
    },
    {
        /* Point dépose rampe arrière */
        .pos = {
            .x = 230,
            .y = 1310,
            .O = 0,        //  0 == dépose cote droit
        },
        .allow_reverse = TRUE,
        .max_speed = MAX_SPEED,
        .act = app_back_ramp_left_drop,
    },
    /*
     * Déplacement vers petit distributeur & pousse des palets en zone de départ
     */
    {
        /* Post-Point récolte 3 palets éloignés balance + orientation vers zone départ */
        .pos = {
            .x = 900,
            .y = 1300,
            .O = 300,
        },
        .allow_reverse = FALSE,
        .max_speed = MAX_SPEED, //MAX_SPEED,
        .act = app_arms_open,
    },
    {
        /* Point interne à la zone de départ */
        .pos = {
            .x = 1100,
            .y = 1000,
            .O = 300
        },
        .allow_reverse = FALSE,
        .max_speed = MAX_SPEED,
        .act = app_arms_close,
    },
    {
        /* Point approche dans couloir vers le petit distributeur */
        .pos = {
            .x = 1275,
            .y = 1500,
            .O = 90,
        },
        .allow_reverse = TRUE,
        .max_speed = MAX_SPEED,
        .act = NULL,
    },
    /*
     * Récolte petit distributeur
     */
    {
        /* Pré-Point */
        .pos = {
            .x = 1275,
            .y = 1750,
            .O = 90,
        },
        .allow_reverse = FALSE,
        .max_speed = NORMAL_SPEED,
        .act = app_front_cup_take,
    },
    {
        /* Point petit distributeur */
        .pos = {
            .x = 1275,
            .y = 1820,
            .O = 90,
        },
        .allow_reverse = FALSE,
        .max_speed = LOW_SPEED,
        .act = NULL,
    },
    {
        /* Post-Point */
        .pos = {
            .x = 1275,
            .y = 1300,
            .O = 90,
        },
        .allow_reverse = TRUE,
        .max_speed = NORMAL_SPEED,
        // TODO: /!\ à symétriser fonction de la couleur
        .act = app_front_cup_ramp,
    },
    {
        /* Post-Point failsafe pump */
        .pos = {
            .x = 1275,
            .y = 1200,
            .O = 270,
        },
        .allow_reverse = TRUE,
        .max_speed = MAX_SPEED,
        .act = app_stop_pumps,
    },
    /*
     * Ouverture du goldenium
     */
    {   /* Pré-point */
        .pos = {
            .x = -150,
            .y = 285,
            .O = 180,
        },
        .allow_reverse = FALSE,
        .max_speed = MAX_SPEED,
        .act = app_back_ramp_left_horiz_for_goldenium,
    },

    {   /* Point ouverture goldenium */
        .pos = {
            .x = -280,
            .y = 285,
            .O = 180,
        },
        .allow_reverse = FALSE,
        .max_speed = NORMAL_SPEED,
        .act = NULL,
    },

    {   /* Post-point */
        .pos = {
            .x = -150,
            .y = 285,
            .O = 180,
        },
        .allow_reverse = TRUE,
        .max_speed = NORMAL_SPEED,
        .act = app_back_ramp_reset,
    },
    /*
     * Récolte goldenium
     */
    {   /* Pré-point */
        .pos = {
            .x = -625,
            .y = 320,
            .O = 270,
        },
        .allow_reverse = TRUE,
        .max_speed = NORMAL_SPEED,
        .act = app_goldenium_take,
    },
    {   /* récolte */
        .pos = {
            .x = -625,
            .y = 250,
            .O = 270,
        },
        .allow_reverse = FALSE,
        .max_speed = LOW_SPEED,
        .act = NULL,
    },
    {   /* Post-point */
        .pos = {
            .x = -625,
            .y = 320,
            .O = 270,
        },
        .allow_reverse = TRUE,
        .max_speed = LOW_SPEED,
        .act = app_goldenium_hold,
    },
    /*
     * Vidange balance avant
     */
    /*
     * Poussette zone de chaos
     */
    {   /* pré point poussette */
        .pos = {
            .x = 90,
            .y = 800,
            .O = 90,
        },
        .allow_reverse = TRUE,
        .max_speed = MAX_SPEED,
        .act = NULL,
    },
    {   /* pré point poussette */
        .pos = {
            .x = 100,
            .y = 1150,
            .O = 340,
        },
        .allow_reverse = FALSE,
        .max_speed = NORMAL_SPEED,
        .act = app_arms_open,
    },
    {   /* pré point poussette */
        .pos = {
            .x = 1000,
            .y = 800,
            .O = 340,
        },
        .allow_reverse = FALSE,
        .max_speed = MAX_SPEED,
        .act = app_arms_close,
    },
    {   /* vidange back ramp */
        .pos = {
            .x = 1000,
            .y = 800,
            .O = 90,
        },
        .allow_reverse = FALSE,
        .max_speed = NORMAL_SPEED,
        .act = app_front_ramp_right_drop,
    },
    {   /* vidange goldenium */
        .pos = {
            .x = 1000,
            .y = 800,
            .O = 0,
        },
        .allow_reverse = FALSE,
        .max_speed = NORMAL_SPEED,
        .act = app_goldenium_drop,
    },

    /*
     * End of path
     */
};

path_t robot_path = {
    /* static cfg */
    .current_pose_idx = 0,
    .play_in_loop = FALSE,
    .nb_pose = sizeof(poses) / sizeof(path_pose_t),
    .poses = poses,
};

inline const path_pose_t *path_get_current_path_pos(const path_t *path)
{
    return &path->poses[path->current_pose_idx];
}

inline const path_pose_t *path_get_pose_at_idx(path_t *path, uint8_t idx)
{
    if ((idx < path->nb_pose) && (idx > 0)) {
        return &path->poses[idx];
    }
    else {
        return NULL;
    }
}

inline uint8_t path_get_current_pose_idx(path_t *path)
{
    return path->current_pose_idx;
}

inline void path_set_current_pose_idx(path_t *path, uint8_t idx)
{
    path->current_pose_idx = idx;
}

inline void path_reset_current_pose_idx(path_t *path)
{
    path->current_pose_idx = 0;
}

inline void path_increment_current_pose_idx(path_t *path)
{
    if (path->current_pose_idx < path->nb_pose - 1) {
        path->current_pose_idx += 1;
    }
    else if (path->play_in_loop) {
        path->current_pose_idx = 0;
    }
}

inline void path_decrement_current_pose_idx(path_t *path)
{
    if (path->current_pose_idx > 0) {
        path->current_pose_idx -= 1;
    }
    else if (path->play_in_loop) {
        path->current_pose_idx = path->nb_pose - 1;
    }
}

inline uint8_t path_get_current_max_speed(const path_t *path)
{
    const path_pose_t *current_path_pos = &path->poses[path->current_pose_idx];

    return MIN(current_path_pos->max_speed, MAX_SPEED);
}

void path_horizontal_mirror_all_pos(const path_t *path)
{
    for (int i = 0; i < path->nb_pose; i++) {
        pose_t *pos = (pose_t *)&path->poses[i].pos;

        pos->x *= -1;
        pos->O = 180 - pos->O;
        pos->O = ((int)pos->O) % 360;
    }
}
