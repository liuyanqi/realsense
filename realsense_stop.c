#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include <lcm/lcm.h>
#include <librealsense/rs.h>
#include <librealsense/rsutil.h>

#include "common/doubles.h"
#include "common/floats.h"
#include "common/getopt.h"
#include "common/image_u8x3.h"
#include "common/time_util.h"
#include "common/zarray.h"
#include "common/zqueue.h"
#include "common/gridmap.h"
#include "lcmtypes/realsense_t.h"
#include "vx/vx.h"
#include "vx/webvx.h"

#define GM_SIZE_Y_M 1
#define GM_SIZE_X_M 2
#define GM_MPP 0.05

typedef struct
{
    lcm_t *lcm;
    vx_world_t *vw;
    webvx_t *webvx;
} state_t;

static void on_create_canvas(vx_canvas_t *vc, const char *name, void *impl)
{
    state_t *state = impl;
    vx_layer_t *vl = vx_canvas_get_layer(vc, "default");
    vx_layer_set_world(vl, state->vw);
}

static void on_destroy_canvas(vx_canvas_t *vc, void *impl)
{

}

typedef struct {
    double normal[3];
    double offset;
} plane_t;

//global variable
int width;
int height;


double curr_x;
double curr_y;

// Plane fit using 3 points
plane_t plane_fit(double pa[3], double pb[3], double pc[3])
{
    plane_t plane;

    double vec_ba[3], vec_ca[3];
    doubles_subtract(pb, pa, 3, vec_ba);
    doubles_subtract(pc, pa, 3, vec_ca);
    doubles_cross_product(vec_ba, vec_ca, plane.normal);
    doubles_normalize(plane.normal, 3, plane.normal);
    plane.offset = doubles_dot(plane.normal, pa, 3);

    return plane;
}

double plane_distance(plane_t *plane, double point[3])
{
    return fabs(plane->offset - doubles_dot(plane->normal, point, 3));
}

// points is a flattened array of [x1 y1 z1 ...]
plane_t plane_fit_ransac(float *points, int* pt2px, int npoints, double threshold, int iters)
{
    int best_count = 0;
    plane_t best_plane = {};

    int center_x = width/2;
    int center_y = height-1;
    for (int i = 0; i < iters; i += 1) {
        int a, b, c;
        //choose the point within certain range of the robot
        do {a = rand() % npoints;} while(pt2px[a] % width < center_x -10 || pt2px[a] % width > center_x+10 || pt2px[a] / width < center_y - 20);
        do { b = rand() % npoints; } while (a == b || pt2px[b] % width < center_x -10 || pt2px[b] % width > center_x+10 || pt2px[b] / width < center_y - 20);
        do { c = rand() % npoints; } while (a == c || b == c || pt2px[c] % width < center_x -10 || pt2px[c] % width > center_x+10 || pt2px[c] / width < center_y - 20);

        double pa[3], pb[3], pc[3];
        for (int j = 0; j < 3; j += 1) {
            pa[j] = points[3*a+j];
            pb[j] = points[3*b+j];
            pc[j] = points[3*c+j];
        }
        plane_t plane = plane_fit(pa, pb, pc);

        int count = 0;
        for (int j = 0; j < npoints; j += 1) {
            double p[3] = {points[3*j], points[3*j+1], points[3*j+2]};

            if (plane_distance(&plane, p) <= threshold)
                count += 1;

            if (best_count < count) {
                best_count = count;
                best_plane = plane;
            }
        }
    }

    return best_plane;
}



static void realsense_handler(const struct _lcm_recv_buf_t *rbuf,
        const char *channel, const struct _realsense_t *msg, void *user)
{
    state_t *state = user;

    width = msg->depth.width;
    height = msg->depth.height;
    float depth_scale = msg->depth_scale;

    rs_intrinsics depth_intrinsics;
    depth_intrinsics.width = width;
    depth_intrinsics.height = height;
    depth_intrinsics.ppx = msg->depth_intrinsics.ppx;
    depth_intrinsics.ppy = msg->depth_intrinsics.ppy;
    depth_intrinsics.fx = msg->depth_intrinsics.fx;
    depth_intrinsics.fy = msg->depth_intrinsics.fy;
    depth_intrinsics.model = msg->depth_intrinsics.model;
    floats_copy(msg->depth_intrinsics.coeffs, depth_intrinsics.coeffs, 5);

    rs_intrinsics color_intrinsics;
    color_intrinsics.width = msg->color.width;
    color_intrinsics.height = msg->color.height;
    color_intrinsics.ppx = msg->color_intrinsics.ppx;
    color_intrinsics.ppy = msg->color_intrinsics.ppy;
    color_intrinsics.fx = msg->color_intrinsics.fx;
    color_intrinsics.fy = msg->color_intrinsics.fy;
    color_intrinsics.model = msg->color_intrinsics.model;
    floats_copy(msg->color_intrinsics.coeffs, color_intrinsics.coeffs, 5);

    rs_extrinsics depth_to_color;
    floats_copy(msg->rotation, depth_to_color.rotation, 9);
    floats_copy(msg->translation, depth_to_color.translation, 3);

    uint16_t *depth_buf = (uint16_t *)msg->depth.data;
    uint8_t *color_buf = (uint8_t *)msg->color.data;

    float *pts = malloc(3*width*height*sizeof(float));
    float *colors = malloc(3*width*height*sizeof(float));
    int npts = 0;
    int *px2pt = calloc(width*height, sizeof(int));
    int *pt2px = calloc(width*height, sizeof(int));
	zarray_t* drivable_region = zarray_create(sizeof(int));
    zarray_t* region = zarray_create(sizeof(int));

    for (int y = 0; y < height; y += 1) {
        for (int x = 0; x < width; x += 1) {
            px2pt[y*width+x] = npts;
            pt2px[npts] = y*width+x;

            float px[2] = {x, y};
            float depth = depth_buf[y*width+x] * depth_scale;
            if (depth == 0.0){
                continue;
			}

            rs_deproject_pixel_to_point(&pts[3*npts], &depth_intrinsics,
                    px, depth);
            float tmp[3];
            rs_transform_point_to_point(tmp, &depth_to_color, &pts[3*npts]);
            rs_project_point_to_pixel(px, &color_intrinsics, tmp);
            int cx = (int)px[0],
            cy = (int)px[1];
            if (cx < 0 || cx >= color_intrinsics.width ||
                    cy < 0 || cy >= color_intrinsics.height) {
                colors[3*npts + 0] = 1.0;
                colors[3*npts + 1] = 1.0;
                colors[3*npts + 2] = 1.0;
            } else {
                int offset = 3 * (cy * color_intrinsics.width + cx);
                colors[3*npts + 0] = color_buf[offset + 0] / 255.0;
                colors[3*npts + 1] = color_buf[offset + 1] / 255.0;
                colors[3*npts + 2] = color_buf[offset + 2] / 255.0;
            }
            npts += 1;
        }
    }

	

    const double threshold = 0.1;
    plane_t ground = plane_fit_ransac(pts, pt2px, npts, threshold, 100);
    for (int i = 0; i < npts; i += 1) {
        double p[3] = {pts[3*i], pts[3*i+1], pts[3*i+2]};
        double dist = plane_distance(&ground, p);
        if (dist <= threshold) {
            colors[3*i] = 0xff;
            colors[3*i + 1] = 0;
            colors[3*i + 2] = 0;
        }
    }

    bool *visited = calloc(width*height, sizeof(bool));
    bool found = false;
    zqueue_t *queue = zqueue_create(sizeof(int));
    {
        int px = 0;
        for(int i =-5; i<5; i++){
            for(int j =0; j<10; j++){
            px = (height-1-j)*(width)+(width+i)/2;
            float depth = depth_buf[px] * depth_scale;
               if(depth != 0.0 && !found){
                    int center_pt = px2pt[px];
                    curr_x = pts[3*center_pt]-1;
                    curr_y = pts[3*center_pt+1];
                    found = true;
               }
            zqueue_push(queue, &px);
            visited[px] = 1;
            }
        }
    }

    //set up gridmap for region within 2m

    double x0 = curr_x;
    double y0 = curr_y;

    printf("center_x: %lf, center_y: %lf \n", x0+1, y0);

     grid_map_t *gm = gridmap_make_meters(x0,
                                         y0,
                                         GM_SIZE_X_M,
                                         GM_SIZE_Y_M,
                                         GM_MPP,
                                         (uint8_t)GRID_VAL_UNKNOWN); 
    double x_max =0;
    double y_max =0;
    double x_min =10;
    double y_min =10;

    while (zqueue_size(queue) > 0) {
        int px;
        zqueue_pop(queue, &px);
        int x = px % width;
        int y = px / width;
        int pt = px2pt[px];

        for (int dy = -2; dy <= 2; dy += 1) {
            if (y+dy < 0 || y+dy >=height)
                continue;

            for (int dx = -2; dx <= 2; dx += 1) {
                if (x+dx < 0 || x+dx >= width)
                    continue;

                int neighbor = x + dx + (y+dy)*width;
                if (visited[neighbor])
                    continue;

                int npt = px2pt[neighbor];

                // 3D distance from point to neighbor
                float dist = floats_distance(&pts[3*pt], &pts[3*npt], 3);

                if (colors[3*npt] == 0xFF && colors[3*npt+2] == 0 &&
                        dist < 0.05) {
                    colors[3*npt+1] = 0xFF;

                    int px_temp = (y+dy)*width+(dx+x);
                    float depth = depth_buf[px_temp] * depth_scale;

                    zarray_add(region, &px_temp);
                    //for everything within 2m region
                    if(depth < 2){
                        zarray_add(drivable_region, &npt);
                        int pt_temp = px2pt[px_temp];
                        if(pts[3*pt_temp] > x_max){
                            x_max = pts[3*pt_temp];
                        }
                        if(pts[3*pt_temp] < x_min){
                            x_min = pts[3*pt_temp];
                        }
                        if(pts[3*pt_temp+1] > y_max){
                            y_max = pts[3*pt_temp+1];
                        }
                        if(pts[3*pt_temp+1] < y_min){
                            y_min = pts[3*pt_temp+1];
                        }


                        gridmap_set_value(gm, pts[3*pt_temp], pts[3*pt_temp+1], (uint8_t) GRID_VAL_TRAVERSABLE);

                        int index_x = gridmap_get_index_x(gm, pts[3*pt_temp]);
                        int index_y = gridmap_get_index_y(gm, pts[3*pt_temp+1]);


                        //colors[3*npt+2] = 0xFF;
                    }

                    visited[neighbor] = 1;
                    zqueue_push(queue, &neighbor);
                }
            }
        }
    }

    printf("x_min: %lf, x_max: %lf, y_min: %lf, y_max: %lf \n", x_min, x_max, y_min, y_max);


    int total_grid =0;

    int counter2 = 0;
    for(double x = x0; x< x0+2; x+=GM_MPP){
        for(double y = y0; y > y0-1; y = y-GM_MPP){
            int cell = gridmap_get_value_safe(gm, x, y, 0);
            if(cell == GRID_VAL_TRAVERSABLE){
                counter2++;
            }
        total_grid++;
        }
    }

    //identify as drivable
    if(counter2 > 300){
        for(int i =0; i< zarray_size(drivable_region); i++){
            int npt = 0;
            zarray_get(drivable_region, i, &npt);
            colors[3*npt] = 0;
            colors[3*npt+1] = 0xFF;
            colors[3*npt+2] = 0;
        }
    }



    printf("counter: %d total grid %d\n",counter2, total_grid);

    free(visited);
    zqueue_destroy(queue);
	zarray_destroy(drivable_region);
    gridmap_destroy(gm);

    // Draw point cloud
    {
        vx_buffer_t *vb = vx_world_get_buffer(state->vw, "points");
        vx_resource_t *points_resc = vx_resource_make_attr_f32_copy(pts,
                3*npts, 3);
        vx_resource_t *colors_resc = vx_resource_make_attr_f32_copy(colors,
                3*npts, 3);
        vx_buffer_add_back(vb,
                vxo_matrix_rotatex(M_PI),
                vxo_points_rgb(points_resc, colors_resc, 1),
                NULL);
        vx_buffer_swap(vb);
    }
    // Draw RGB image in corner
    {
        image_u8x3_t render_im = {.width = width, .height = height,
            .stride = 3*width, .buf = color_buf};

        for(int i =0; i< zarray_size(region); i++){
            int px = 0;
            zarray_get(region, i, &px);
            int x_temp = px%width;
            int y_temp = px/width;

            render_im.buf[y_temp*render_im.stride+3*x_temp+0] = 255;
            
        }
       
        vx_buffer_t *vb = vx_world_get_buffer(state->vw, "rgb-image");
        vx_buffer_add_back(vb,
                vxo_pixcoords(VXO_PIXCOORDS_TOP_LEFT,
                    VXO_PIXCOORDS_SCALE_MODE_ONE,
                    vxo_matrix_scale3(1, -1, 1),
                    vxo_image_u8x3(&render_im, 0),
                    NULL),
                NULL);
        vx_buffer_swap(vb);
    }

    free(pts);
    free(colors);
    free(pt2px);
    free(px2pt);
}

int main(int argc, char *argv[])
{
    state_t *state = calloc(1, sizeof(state_t));

    getopt_t *gopt = getopt_create();
    getopt_add_bool(gopt, 'h', "help", 0, "Print this help");
    getopt_add_int(gopt, 'p', "port", "7767", "Port for jsvx");
    getopt_add_string(gopt, 'c', "channel", "REALSENSE", "Realsense channel");
    if (!getopt_parse(gopt, argc, argv, 0) || getopt_get_bool(gopt, "help")) {
        getopt_do_usage(gopt);
        return -1;
    }

    state->lcm = lcm_create(NULL);
    realsense_t_subscribe(state->lcm, getopt_get_string(gopt, "channel"),
            realsense_handler, state);

    // Vx init
    int port = getopt_get_int(gopt, "port");
    state->vw = vx_world_create();
    state->webvx = webvx_create_server(port, NULL, "index.html");
    webvx_define_canvas(state->webvx, "mycanvas", on_create_canvas,
                        on_destroy_canvas, state);
    printf("jsvx server started on port %d\n", port);
    {
        vx_buffer_t *vb = vx_world_get_buffer(state->vw, "grid");
        vx_buffer_add_back(vb,
                vxo_grid((float[]) { .5, .5, .5, .5 }, 1),
                NULL);
        vx_buffer_swap(vb);
    }

    while (1) {
        lcm_handle(state->lcm);
    }
}
