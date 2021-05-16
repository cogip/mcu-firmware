#include <stdio.h>
#include <kernel_defines.h>

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
            .coords.x = 1500 - ROBOT_MARGIN,
            .coords.y = 300 + ROBOT_MARGIN,
            .O = 180,
        },
        .allow_reverse = FALSE,
        .max_speed = MAX_SPEED,
        .act = NULL,
    },
    /* Game path */
    {                           /* Sortie zone de départ */
        .pos = {
            .coords.x = 1275,   //1100,
            .coords.y = 300 + ROBOT_MARGIN,
            .O = 90,
        },
        .allow_reverse = FALSE,
        .max_speed = NORMAL_SPEED,
        .act = NULL,
    },
    /* Point évitement zone du chaos x=800 y=1100 O=205+90*/
    {
        .pos = {
            .coords.x = 1275,           //800,
            .coords.y = 1300,           //1100,
            .O = 115,
        },
        .allow_reverse = FALSE,
        .max_speed = NORMAL_SPEED,
        .act = NULL,
    },
    {
        .pos = {
            .coords.x = 600,            //800,
            .coords.y = 1300,           //1100,
            .O = 90,
        },
        .allow_reverse = FALSE,
        .max_speed = NORMAL_SPEED,
        .act = NULL,
    },
    {
        /* Pré-Point */
        .pos = {
            .coords.x = 600,
            .coords.y = 1300,
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
            .coords.x = 600,
            .coords.y = 1361,
            .O = 90,
        },
        .allow_reverse = FALSE,
        .max_speed = LOW_SPEED,
        .act = NULL,
    },
    {
        /* Post-Point */
        .pos = {
            .coords.x = 600,
            .coords.y = 1300,
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
            .coords.x = 900,
            .coords.y = 1300,
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
            .coords.x = 900,
            .coords.y = 1361,
            .O = 270,
        },
        .allow_reverse = TRUE,
        .max_speed = LOW_SPEED,
        .act = NULL,
    },
    {
        /* Post-Point */
        .pos = {
            .coords.x = 900,
            .coords.y = 1300,
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
            .coords.x = 230,
            .coords.y = 1310,
            .O = 180,        // 180: dépose cote gauche
        },
        .allow_reverse = FALSE,
        .max_speed = MAX_SPEED, //MAX_SPEED,
        .act = app_front_ramp_right_drop,
    },
    {
        /* Point dépose rampe arrière */
        .pos = {
            .coords.x = 230,
            .coords.y = 1310,
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
            .coords.x = 900,
            .coords.y = 1300,
            .O = 300,
        },
        .allow_reverse = FALSE,
        .max_speed = MAX_SPEED, //MAX_SPEED,
        .act = app_arms_open,
    },
    {
        /* Point interne à la zone de départ */
        .pos = {
            .coords.x = 1100,
            .coords.y = 1000,
            .O = 300
        },
        .allow_reverse = FALSE,
        .max_speed = MAX_SPEED,
        .act = app_arms_close,
    },
    {
        /* Point approche dans couloir vers le petit distributeur */
        .pos = {
            .coords.x = 1275,
            .coords.y = 1500,
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
            .coords.x = 1275,
            .coords.y = 1750,
            .O = 90,
        },
        .allow_reverse = FALSE,
        .max_speed = NORMAL_SPEED,
        .act = app_front_cup_take,
    },
    {
        /* Point petit distributeur */
        .pos = {
            .coords.x = 1275,
            .coords.y = 1820,
            .O = 90,
        },
        .allow_reverse = FALSE,
        .max_speed = LOW_SPEED,
        .act = NULL,
    },
    {
        /* Post-Point */
        .pos = {
            .coords.x = 1275,
            .coords.y = 1300,
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
            .coords.x = 1275,
            .coords.y = 1200,
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
            .coords.x = -150,
            .coords.y = 285,
            .O = 180,
        },
        .allow_reverse = FALSE,
        .max_speed = MAX_SPEED,
        .act = app_back_ramp_left_horiz_for_goldenium,
    },

    {   /* Point ouverture goldenium */
        .pos = {
            .coords.x = -280,
            .coords.y = 285,
            .O = 180,
        },
        .allow_reverse = FALSE,
        .max_speed = NORMAL_SPEED,
        .act = NULL,
    },

    {   /* Post-point */
        .pos = {
            .coords.x = -150,
            .coords.y = 285,
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
            .coords.x = -625,
            .coords.y = 320,
            .O = 270,
        },
        .allow_reverse = TRUE,
        .max_speed = NORMAL_SPEED,
        .act = app_goldenium_take,
    },
    {   /* récolte */
        .pos = {
            .coords.x = -625,
            .coords.y = 250,
            .O = 270,
        },
        .allow_reverse = FALSE,
        .max_speed = LOW_SPEED,
        .act = NULL,
    },
    {   /* Post-point */
        .pos = {
            .coords.x = -625,
            .coords.y = 320,
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
            .coords.x = 90,
            .coords.y = 800,
            .O = 90,
        },
        .allow_reverse = TRUE,
        .max_speed = MAX_SPEED,
        .act = NULL,
    },
    {   /* pré point poussette */
        .pos = {
            .coords.x = 100,
            .coords.y = 1150,
            .O = 340,
        },
        .allow_reverse = FALSE,
        .max_speed = NORMAL_SPEED,
        .act = app_arms_open,
    },
    {   /* pré point poussette */
        .pos = {
            .coords.x = 1000,
            .coords.y = 800,
            .O = 340,
        },
        .allow_reverse = FALSE,
        .max_speed = MAX_SPEED,
        .act = app_arms_close,
    },
    {   /* vidange back ramp */
        .pos = {
            .coords.x = 1000,
            .coords.y = 800,
            .O = 90,
        },
        .allow_reverse = FALSE,
        .max_speed = NORMAL_SPEED,
        .act = app_front_ramp_right_drop,
    },
    {   /* vidange goldenium */
        .pos = {
            .coords.x = 1000,
            .coords.y = 800,
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
    .nb_poses = ARRAY_SIZE(poses),
    .poses = poses,
};
