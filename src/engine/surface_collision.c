#include <PR/ultratypes.h>
#include "sm64.h"
#include "game/debug.h"
#include "game/level_update.h"
#include "game/mario.h"
#include "game/object_list_processor.h"
#include "surface_collision.h"
#include "surface_load.h"

#define gaiseki(a, b, c, d, e, f) (d - b) * (e - c) - (c - a) * (f - d) // taken from TPP
#define Mabs(x)                                ((x) < 0 ? (-(x)) : (x))
#define WALLPLANELIST_MAX              (4)
#define MAP_LIMIT_MIN           (-8192)
#define MAP_LIMIT_MAX           (+8192)
#define MAP_HALF_SIZE           (+8192)
#define MAP_AREA_SIZE           CELL_SIZE
#define _CHECK_WALL             SPATIAL_PARTITION_WALLS
#define _CHECK_ROOF             SPATIAL_PARTITION_CEILS
#define _CHECK_GROUND           SPATIAL_PARTITION_FLOORS
#define MAX_speedY              (-78)
#define roofnull_height         CELL_HEIGHT_LIMIT
#define null_height             FLOOR_LOWER_LIMIT
#define DBF_BGnull              gNumFindFloorMisses
#define waterline               gEnvironmentRegions
// STRUCT DEFINES
typedef struct SurfaceNode BGCheckList;
typedef struct Surface BGCheckData;
typedef struct WallCollisionData WallCheckRecord;
typedef struct FloorGeometry Plane; // ?!?
// STRUCT MEMBER DEFINES
#define L_wx   x                                       // WallCheckRecord::wx
#define L_wy   y                                       // WallCheckRecord::wy
#define L_wz   z                                       // WallCheckRecord::wz
#define L_walllistptr  numWalls        // WallCheckRecord::walllistptr
#define L_wall walls                           // WallCheckRecord::wall
#define L_r            radius                          // WallCheckRecord::r
#define L_data surface                         // BGCheckList::data
#define L_a2   normalX                         // Plane::a
#define L_b2   normalY                         // Plane::b
#define L_c2   normalZ                         // Plane::c
#define L_d2   originOffset            // Plane::d
#define L_a            normal.x                        // BGCheckData::a
#define L_b            normal.y                        // BGCheckData::b
#define L_c            normal.z                        // BGCheckData::c
#define L_d            originOffset            // BGCheckData::d
#define L_x1   vertex1[0]                      // BGCheckData below
#define L_y1   vertex1[1]
#define L_z1   vertex1[2]
#define L_x2   vertex2[0]
#define L_y2   vertex2[1]
#define L_z2   vertex2[2]
#define L_x3   vertex3[0]
#define L_y3   vertex3[1]
#define L_z3   vertex3[2]
// CODE LINKS
#define BG_WallCheck           find_wall_collisions_from_list
#define WallCheck                      f32_find_wall_collision
#define mcWallCheck                    find_wall_collisions
#define BG_RoofCheck           find_ceil_from_list
#define mcBGRoofCheck          find_ceil
#define BG_GroundCheck         find_floor_from_list
#define BGcheck                                find_floor_height
#define mcMoveGroundCheck      unused_find_dynamic_floor
#define mcBGGroundCheck                find_floor
#define mcGroundCheck          find_floor_height_and_data
#define mcWaterCheck           find_water_level
static Plane hit_mapplane;
static Plane hit_roofplane;

/**************************************************
 *                      WALLS                     *
 **************************************************/

int BG_WallCheck(BGCheckList* bgcheck_list, WallCheckRecord* wallcheckp) // aka find_wall_collisions_from_list
{

//     int counter;
       float   A,B,C,D,dR,dRf;
       float   x1,x2,x3,y1,y2,y3;

       float   px  =  wallcheckp->L_wx;
       float   py  = (wallcheckp->L_wy) + (wallcheckp->offsetY);
       float   pz  =  wallcheckp->L_wz;
       float   r   =  wallcheckp->L_r;

       int             wallflag = 0;                   /* hit OFF/ON 0/1 */
       int             g_flag;
       BGCheckData             *wall;                          /* bgcheck data */



       while( bgcheck_list != NULL ){

//             counter++;

               wall             = bgcheck_list->L_data;
               bgcheck_list = bgcheck_list->next;

               if ( wall->L_a < -0.707 || 0.707 < wall->L_a){
                       x1 = -(wall->L_z1);     x2 = -(wall->L_z2);     x3 = -(wall->L_z3);
                       y1 =   wall->L_y1;      y2 =   wall->L_y2;      y3 =   wall->L_y3;
                       g_flag = 0;
                       if ( gaiseki(-pz,py,x1,y1,x2,y2) > 0 )  g_flag++;
                       if ( gaiseki(-pz,py,x2,y2,x3,y3) > 0 )  g_flag++;
                       if ( gaiseki(-pz,py,x3,y3,x1,y1) > 0 )  g_flag++;
                       if ( g_flag == 1 || g_flag == 2 ) continue;

               } else {
                       x1 = wall->L_x1;                x2 = wall->L_x2;                x3 = wall->L_x3;
                       y1 = wall->L_y1;                y2 = wall->L_y2;                y3 = wall->L_y3;
                       g_flag = 0;
                       if ( gaiseki(px,py,x1,y1,x2,y2) > 0 ) g_flag++;
                       if ( gaiseki(px,py,x2,y2,x3,y3) > 0 ) g_flag++;
                       if ( gaiseki(px,py,x3,y3,x1,y1) > 0 ) g_flag++;
                       if ( g_flag == 1 || g_flag == 2 ) continue;
               }

               A = wall->L_a;
               B = wall->L_b;
               C = wall->L_c;
               D = wall->L_d;

               dR  = A*px+B*py+C*pz+D;
               dRf = Mabs(dR);


               if ( dRf < r ){

//                     dbPrint("dr %d",(int)dR );
//                     rmonPrintf("A%f,B%f,C%f,%f\n",A,B,C,dR);

                       wallcheckp->L_wx += A*(r-dR);           /* Xposition offset     */
                       wallcheckp->L_wz += C*(r-dR);   /* Zposition offset             */

                       if ( wallcheckp->L_walllistptr < WALLPLANELIST_MAX ){

                               //wallcheckp->L_wall[wallcheckp->L_walllistptr] = &Xwall[wallcheckp->L_walllistptr]; ---- what are you doing man
                               //Xwall[wallcheckp->L_walllistptr].a = wall->L_a;
                               //Xwall[wallcheckp->L_walllistptr].b = wall->L_b;
                               //Xwall[wallcheckp->L_walllistptr].c = wall->L_c;
                               //Xwall[wallcheckp->L_walllistptr].d = wall->L_d;
                               wallcheckp->L_wall[wallcheckp->L_walllistptr] = wall;
                               (wallcheckp->L_walllistptr)++;

                       }

                       wallflag++;
               }
       }





       return(wallflag);
}

extern int     mcWallCheck(WallCheckRecord *wall);

int WallCheck(float* wX, float* wY, float* wZ, float offsetY, float r) // aka f32_find_wall_collision
{

       WallCheckRecord wall;
       int             wallcount       = 0;

       wall.offsetY    = offsetY;
       wall.L_r        = r;
       wall.L_wx = *wX;        wall.L_wy = *wY;        wall.L_wz = *wZ;

       wallcount = mcWallCheck(&wall);

       *wX     = wall.L_wx;    *wY     = wall.L_wy;    *wZ     = wall.L_wz;

       return(wallcount);
}

/*-------------------------------------------------------------------------------*/
extern int     mcWallCheck(WallCheckRecord *wall) // aka find_wall_collisions
{

       BGCheckList     *bgcheck_list;
       short   area_x,area_z;
       int             wallcount       = 0;
       wall->L_walllistptr     = 0;    /* Walllist pointer Reset!! */

       if ( wall->L_wx < MAP_LIMIT_MIN || MAP_LIMIT_MAX < wall->L_wx ) return( wallcount );
       if ( wall->L_wz < MAP_LIMIT_MIN || MAP_LIMIT_MAX < wall->L_wz ) return( wallcount );

#if 1 // N.B. Swap these to fix the "thwomps in WF can push Mario through static walls" bug
       /*--- static ----*/
       area_x                  = ( (wall->L_wx) + MAP_HALF_SIZE ) / 1024;
       area_z                  = ( (wall->L_wz) + MAP_HALF_SIZE ) / 1024;
       bgcheck_list    = bgcheck_arealist[area_z][area_x].root[_CHECK_WALL].next;
       wallcount               += BG_WallCheck(bgcheck_list,wall);
#endif

       /*--- dynamic ---*/
       bgcheck_list    = movebg_head.root[_CHECK_WALL].next;
       wallcount               += BG_WallCheck(bgcheck_list,wall);


       return(wallcount);
}

/**************************************************
 *                     CEILINGS                   *
 **************************************************/

BGCheckData* BG_RoofCheck(BGCheckList* bgcheck_list, int x, int y, int z, float* planeY) // aka find_ceil_from_list
{

       BGCheckData             *bgcheck_data, *bgcheck_hitdata;
       float   x1,z1,x2,z2,x3,z3;              /*      polygon pointdata */
       float   A,B,C,D,roofY;

       bgcheck_hitdata = NULL;

       while ( bgcheck_list != NULL ){

               bgcheck_data = bgcheck_list->L_data;    /* Next DataCheck !! */
               bgcheck_list = bgcheck_list->next;              /* next BGcheck !!   */

               x1 = bgcheck_data->L_x1; x2 = bgcheck_data->L_x2; x3 = bgcheck_data->L_x3;
               z1 = bgcheck_data->L_z1; z2 = bgcheck_data->L_z2; z3 = bgcheck_data->L_z3;

               if ( gaiseki(x,z,x1,z1,x2,z2) > 0 ) continue;
               if ( gaiseki(x,z,x2,z2,x3,z3) > 0 ) continue;
               if ( gaiseki(x,z,x3,z3,x1,z1) > 0 ) continue;

               A = bgcheck_data->L_a;
               B = bgcheck_data->L_b;
               C = bgcheck_data->L_c;
               D = bgcheck_data->L_d;
               roofY = -(A*x+C*z+D)/B;

               if ( y-(roofY-(MAX_speedY)) > 0 )       continue;

               *planeY = roofY;
               bgcheck_hitdata = bgcheck_data;
               break;                                                                  /* Hakken Shita !! */

       }

       return(bgcheck_hitdata);
}

extern float mcRoofCheck(float px, float py, float pz,Plane **hitplane);

float RoofCheck(float px, float py, float pz, Plane** hitplane) // don't know what this is
{

       float   planeY          = roofnull_height;                      /* initialize   */
       *hitplane                       = NULL;                                         /* initilaize   */
       planeY = mcRoofCheck(px,py,pz,hitplane);
       return(planeY);
}

/*-----------------------------------------------------------------------------*/
extern float mcBGRoofCheck(float px, float py, float pz,BGCheckData **bgdata) // aka find_ceil
{

       short           area_z,area_x;
       BGCheckData *bgcheck_data;
       BGCheckData *bgcheck_data_dynamic;
       BGCheckList *bgcheck_list;
       float           planeY                  = roofnull_height;      /* initialize   */
       float           planeY_dynamic  = roofnull_height;      /* initialize   */
       *bgdata         = NULL;


       /*--- Map OverFlow Check ---*/

       if ( px < MAP_LIMIT_MIN || MAP_LIMIT_MAX < px ) return( planeY );
       if ( pz < MAP_LIMIT_MIN || MAP_LIMIT_MAX < pz ) return( planeY );

       /*--- dynamic ---*/
       bgcheck_list             = movebg_head.root[_CHECK_ROOF].next;
       bgcheck_data_dynamic = BG_RoofCheck(bgcheck_list,px,py,pz,&planeY_dynamic);

       /*--- static ----*/
       area_x                          = (short)( ( px + MAP_HALF_SIZE ) / MAP_AREA_SIZE );
       area_z                          = (short)( ( pz + MAP_HALF_SIZE ) / MAP_AREA_SIZE );
       bgcheck_list            = bgcheck_arealist[area_z][area_x].root[_CHECK_ROOF].next;
       bgcheck_data            = BG_RoofCheck(bgcheck_list,px,py,pz,&planeY);

       /*---- height check !! ---*/
       if ( planeY_dynamic < planeY ){
               bgcheck_data    = bgcheck_data_dynamic;
               planeY                  = planeY_dynamic;
       }

       *bgdata = bgcheck_data;

       return(planeY);
}

/*-----------------------------------------------------------------------------*/
extern float mcRoofCheck(float px, float py, float pz,Plane **hitplane) // also don't know what this is
{

       float planeY;
       BGCheckData     *bgcheck_data;
       *hitplane                                       = NULL;                         /* initilaize   */

       planeY = mcBGRoofCheck(px,py,pz,&bgcheck_data);

       if ( bgcheck_data != NULL ){
               hit_roofplane.L_a2 = bgcheck_data->L_a;
               hit_roofplane.L_b2 = bgcheck_data->L_b;
               hit_roofplane.L_c2 = bgcheck_data->L_c;
               hit_roofplane.L_d2 = bgcheck_data->L_d;
               *hitplane = &hit_roofplane;
       }

       return(planeY);
}

/**************************************************
 *                     FLOORS                     *
 **************************************************/

BGCheckData* BG_GroundCheck(BGCheckList* bgcheck_list, int x, int y, int z, float* planeY) // AKA: find_floor_from_list
{

       BGCheckData             *bgcheck_data, *bgcheck_hitdata;
       float   x1,z1,x2,z2,x3,z3;              /*      polygon pointdata */
       float   A,B,C,D,groundY;


       bgcheck_hitdata = NULL;

       while ( bgcheck_list != NULL ){

               bgcheck_data = bgcheck_list->L_data;    /* Next DataCheck !! */
               bgcheck_list = bgcheck_list->next;              /* next BGcheck !!   */

               x1 = bgcheck_data->L_x1; x2 = bgcheck_data->L_x2; x3 = bgcheck_data->L_x3;
               z1 = bgcheck_data->L_z1; z2 = bgcheck_data->L_z2; z3 = bgcheck_data->L_z3;

               if ( gaiseki(x,z,x1,z1,x2,z2) < 0 ) continue;
               if ( gaiseki(x,z,x2,z2,x3,z3) < 0 ) continue;
               if ( gaiseki(x,z,x3,z3,x1,z1) < 0 ) continue;

               A = bgcheck_data->L_a;
               B = bgcheck_data->L_b;
               C = bgcheck_data->L_c;
               D = bgcheck_data->L_d;
               groundY = -(A*x+C*z+D)/B;

               if ( y-(groundY+MAX_speedY) < 0 )       continue;

               *planeY = groundY;
               bgcheck_hitdata = bgcheck_data;
               break;                          /* Hakken Shita !! */

       }

       return(bgcheck_hitdata);
}

extern float mcBGGroundCheck(float px, float py, float pz, BGCheckData** bgdata);
extern float mcGroundCheck(float px, float py, float pz,Plane **hitplane);

float BGcheck(float px, float py, float pz) // AKA: find_floor_height
{
       BGCheckData     *bgdata;
       float   planeY;
       planeY = mcBGGroundCheck(px,py,pz,&bgdata);
       return(planeY);

}

/*-----------------------------------------------------------------------------*/
extern float GroundCheck(float px, float py, float pz,Plane **hitplane) // unsure what this is
{

       float   planeY          = null_height;                          /* initialize   */
       *hitplane                       = NULL;                                         /* initilaize   */
       planeY = mcGroundCheck(px,py,pz,hitplane);
       return(planeY);

}

/*-----------------------------------------------------------------------------*/
extern float mcMoveGroundCheck(float px, float py, float pz,BGCheckData **bgcheck_data)  // AKA: unused_find_dynamic_floor
{

       BGCheckList *bgcheck_list;
       BGCheckData *bgcheck_data_dynamic;
       float           planeY                  = null_height;                          /* initialize   */

       /*--- dynamic ---*/
       bgcheck_list             = movebg_head.root[_CHECK_GROUND].next;
       bgcheck_data_dynamic = BG_GroundCheck(bgcheck_list,px,py,pz,&planeY);

       *bgcheck_data = bgcheck_data_dynamic;

       return(planeY);
}

float mcBGGroundCheck(float px, float py, float pz, BGCheckData** bgdata) // AKA: find_floor
{

       short           area_z,area_x;
       BGCheckData *bgcheck_data;
       BGCheckData *bgcheck_data_dynamic;
       BGCheckList *bgcheck_list;
       float           planeY                  = null_height;                          /* initialize   */
       float           planeY_dynamic  = null_height;                          /* initialize   */
       *bgdata         = NULL;

       if ( px < MAP_LIMIT_MIN || MAP_LIMIT_MAX < px ) return( planeY );
       if ( pz < MAP_LIMIT_MIN || MAP_LIMIT_MAX < pz ) return( planeY );

       /*--- dynamic ---*/
       bgcheck_list             = movebg_head.root[_CHECK_GROUND].next;
       bgcheck_data_dynamic = BG_GroundCheck(bgcheck_list,px,py,pz,&planeY_dynamic);

       /*---- static ---*/
       area_x                           = (short)( ( px + MAP_HALF_SIZE ) / MAP_AREA_SIZE );
       area_z                           = (short)( ( pz + MAP_HALF_SIZE ) / MAP_AREA_SIZE );
       bgcheck_list             = bgcheck_arealist[area_z][area_x].root[_CHECK_GROUND].next;
       bgcheck_data             = BG_GroundCheck(bgcheck_list,px,py,pz,&planeY);

       /*== ERR CHECK ==*/
       if ( bgcheck_data == NULL       )       DBF_BGnull++;

       /*---- height check !! ---*/
       if ( planeY_dynamic > planeY ){
               bgcheck_data    = bgcheck_data_dynamic;
               planeY                  = planeY_dynamic;
       }

       *bgdata = bgcheck_data;











       return(planeY);

}

/*-----------------------------------------------------------------------------*/
extern float mcGroundCheck(float px, float py, float pz,Plane **hitplane) // AKA most likely: find_floor_height_and_data
{

       float planeY;
       BGCheckData     *bgcheck_data;
       *hitplane       = NULL;                                         /* initilaize   */
       planeY = mcBGGroundCheck(px,py,pz,&bgcheck_data);

       if ( bgcheck_data != NULL ){
               /* Set Normal Vector */
               hit_mapplane.L_a2 = bgcheck_data->L_a;
               hit_mapplane.L_b2 = bgcheck_data->L_b;
               hit_mapplane.L_c2 = bgcheck_data->L_c;
               hit_mapplane.L_d2 = bgcheck_data->L_d;
               *hitplane = &hit_mapplane;
       }

       return(planeY);
}

/**************************************************
 *               ENVIRONMENTAL BOXES              *
 **************************************************/

float mcWaterCheck(float wx, float wz) // AKA: find_water_level
{

       int             i,loop;
       UNUSED  short   code;
       float   Xmin,Xmax,Zmin,Zmax;
       float   waterY = -11000;
       short   *water = waterline;

       if ( water != NULL ){

               loop = *water++;

               for(i=0;i<loop;i++){

                       code = *water++;
                       Xmin = *water++;
                       Zmin = *water++;
                       Xmax = *water++;
                       Zmax = *water++;

                       if ( (Xmin<wx) && (wx<Xmax) && (Zmin<wz) && (wz<Zmax) ){
                               waterY = *water;
                               break;
                       }
                       water++;
               }
       }

       return(waterY);
}

//void mcPlaneCheck(float px, float py, float pz) // No idea what this is supposed to be
//{
//
//     Plane   *hitplane;
//     float   bgY = mcGroundCheck(px,py,pz,&hitplane);
//
//     plane->counter = 0;
//
//     if ( hitplane != NULL ){
//
//             plane->counter    = 1;
//             plane->plane[0].a = hitplane->a;
//             plane->plane[0].b = hitplane->b;
//             plane->plane[0].c = hitplane->c;
//             plane->plane[0].d = -(  (hitplane->a)*px + (hitplane->b)*bgY + (hitplane->c)*pz );
//
//     } else {
//
//             plane->plane[0].a = 0.0;
//             plane->plane[0].b = 1.0;
//             plane->plane[0].c = 0.0;
//             plane->plane[0].d = 11000;
//
//     }
//
//
//}

// debug stuff I decided to leave in

/**************************************************
 *                      DEBUG                     *
 **************************************************/

/**
 * Finds the length of a surface list for debug purposes.
 */
static s32 surface_list_length(struct SurfaceNode *list) {
    s32 count = 0;

    while (list != NULL) {
        list = list->next;
        count++;
    }

    return count;
}

/**
 * Print the area,number of walls, how many times they were called,
 * and some allocation information.
 */
void debug_surface_list_info(f32 xPos, f32 zPos) {
    struct SurfaceNode *list;
    s32 numFloors = 0;
    s32 numWalls = 0;
    s32 numCeils = 0;

    s32 cellX = (xPos + LEVEL_BOUNDARY_MAX) / CELL_SIZE;
    s32 cellZ = (zPos + LEVEL_BOUNDARY_MAX) / CELL_SIZE;

    list = bgcheck_arealist[cellZ & NUM_CELLS_INDEX][cellX & NUM_CELLS_INDEX].root[SPATIAL_PARTITION_FLOORS].next;
    numFloors += surface_list_length(list);

    list = movebg_head.root[SPATIAL_PARTITION_FLOORS].next;
    numFloors += surface_list_length(list);

    list = bgcheck_arealist[cellZ & NUM_CELLS_INDEX][cellX & NUM_CELLS_INDEX].root[SPATIAL_PARTITION_WALLS].next;
    numWalls += surface_list_length(list);

    list = movebg_head.root[SPATIAL_PARTITION_WALLS].next;
    numWalls += surface_list_length(list);

    list = bgcheck_arealist[cellZ & NUM_CELLS_INDEX][cellX & NUM_CELLS_INDEX].root[SPATIAL_PARTITION_CEILS].next;
    numCeils += surface_list_length(list);

    list = movebg_head.root[SPATIAL_PARTITION_CEILS].next;
    numCeils += surface_list_length(list);

    print_debug_top_down_mapinfo("area   %x", cellZ * NUM_CELLS + cellX);

    // Names represent ground, walls, and roofs as found in SMS.
    print_debug_top_down_mapinfo("dg %d", numFloors);
    print_debug_top_down_mapinfo("dw %d", numWalls);
    print_debug_top_down_mapinfo("dr %d", numCeils);

    set_text_array_x_y(80, -3);

    print_debug_top_down_mapinfo("%d", gNumCalls.floor);
    print_debug_top_down_mapinfo("%d", gNumCalls.wall);
    print_debug_top_down_mapinfo("%d", gNumCalls.ceil);

    set_text_array_x_y(-80, 0);

    // listal- List Allocated?, statbg- Static Background?, movebg- Moving Background?
    print_debug_top_down_mapinfo("listal %d", gSurfaceNodesAllocated);
    print_debug_top_down_mapinfo("statbg %d", gNumStaticSurfaces);
    print_debug_top_down_mapinfo("movebg %d", gSurfacesAllocated - gNumStaticSurfaces);

    gNumCalls.floor = 0;
    gNumCalls.ceil = 0;
    gNumCalls.wall = 0;
}

/**
 * An unused function that finds and interacts with any type of surface.
 * Perhaps an original implementation of surfaces before they were more specialized.
 */
s32 unused_resolve_floor_or_ceil_collisions(s32 checkCeil, f32 *px, f32 *py, f32 *pz, f32 radius,
                                            struct Surface **psurface, f32 *surfaceHeight) {
    f32 nx, ny, nz, oo;
    f32 x = *px;
    f32 y = *py;
    f32 z = *pz;
    f32 offset, distance;

    *psurface = NULL;

    if (checkCeil) {
        *surfaceHeight = find_ceil(x, y, z, psurface);
    } else {
        *surfaceHeight = find_floor(x, y, z, psurface);
    }

    if (*psurface == NULL) {
        return -1;
    }

    nx = (*psurface)->normal.x;
    ny = (*psurface)->normal.y;
    nz = (*psurface)->normal.z;
    oo = (*psurface)->originOffset;

    offset = nx * x + ny * y + nz * z + oo;
    distance = offset >= 0 ? offset : -offset;

    // Interesting surface interaction that should be surf type independent.
    if (distance < radius) {
        *px += nx * (radius - offset);
        *py += ny * (radius - offset);
        *pz += nz * (radius - offset);

        return 1;
    }

    return 0;
}
