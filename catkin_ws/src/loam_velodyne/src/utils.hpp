#ifndef UTILS_HPP
#define UTILS_HPP

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl/io/ply_io.h>
#include <dirent.h>
#include <opencv2/opencv.hpp>
#include "../include/loam_velodyne/Twist.h"
#include "loam_velodyne/LaserMapping.h"
#include "lib/math_utils.h"
#include "../include/loam_velodyne/common.h"
#include "delaunay-triangulation/vector2.h"
#include "delaunay-triangulation/triangle.h"
#include "delaunay-triangulation/delaunay.h"
#include <thread>

#include <opengm/opengm.hxx>
#include <opengm/graphicalmodel/graphicalmodel.hxx>
#include <opengm/operations/adder.hxx>
#include <opengm/functions/potts.hxx>

#include <opengm/inference/messagepassing/messagepassing.hxx>
#include <opengm/operations/minimizer.hxx>
#include <opengm/inference/icm.hxx>
#include<opengm/inference/external/mrflib.hxx>
#include "cnpy.h"
#include "../bp-vision/image.h"

#define win_size 11
#define VALUES win_size*win_size

int PATCH_SIZE = 20;

typedef double ValueType;          // type used for values
typedef size_t IndexType; // type used for indexing nodes and factors (default : size_t)
typedef size_t LabelType;          // type used for labels (default : size_t)
typedef opengm::Adder OpType;             // operation used to combine terms
typedef opengm::ExplicitFunction <ValueType, IndexType, LabelType> ExplicitFunction; // shortcut for explicit function
typedef opengm::PottsFunction <ValueType, IndexType, LabelType> PottsFunction; // shortcut for Potts function
typedef opengm::meta::TypeListGenerator<ExplicitFunction, PottsFunction>::type FunctionTypeList; // list of all function the model can use (this trick avoids virtual methods) - here only one
typedef opengm::DiscreteSpace <IndexType, LabelType> SpaceType; // type used to define the feasible state-space
typedef opengm::GraphicalModel <ValueType, OpType, FunctionTypeList, SpaceType> Model; // type of the model
typedef Model::FunctionIdentifier FunctionIdentifier; // type of the function identifier
typedef opengm::GraphicalModel<double, opengm::Adder> GraphicalModelType;
typedef opengm::external::MRFLIB<GraphicalModelType> MRFLIB;
struct PixLabelColors {
    double ts;
    std::vector<float> pix_label_costs;
    std::vector <cv::Vec3b> pix_colors;
    std::vector <cv::Point2f> pix_locs;
    std::vector <cv::Vec3b> left_colors;
    std::vector <cv::Vec3b> right_colors;
    std::vector <cv::Vec3b> top_colors;
    std::vector <cv::Vec3b> bot_colors;
};

std::map<cv::Mat *, cv::Mat> img2depth;
std::map<double, cv::Mat> time2depth;
std::map<double, cv::Mat> time2forwardflow;
std::map<double, cv::Mat> time2backwardflow;

double color_smooth_weight = 2.0;
const float bound_weight = 50.f;
//const float proj_color_weight = 5.f;
const float center_color_weight = 30.f;


struct PointXYZIS {
    PointXYZIS() {
    }

    PointXYZIS(float ix, float iy, float iz, float inten, float rp) :
            x(ix), y(iy), z(iz), intensity(inten), rotate_percentage(rp) {
    }

    float x, y, z;
    float intensity;
    float rotate_percentage;
    double lidar_frame_time = 0;

    cv::Mat *img_ptr = nullptr;
    int u, v;
    double to_img_distance = 0;
    double img_frame_time = 0;
};

template <class T>
bool getValueSubpix(const cv::Mat& uv_map, cv::Point2f pix, T &uv)
{
    if (0<pix.x && pix.x < uv_map.cols -1 && 0<pix.y && pix.y < uv_map.rows -1) {
        cv::Mat patch;
        cv::getRectSubPix(uv_map, cv::Size(1, 1), pix, patch);
        uv = patch.at<T>(0, 0);
        return true;
    }else{
        return false;
    }
}

cv::Point2f findPixOffset(cv::Point2f p1, double t1, cv::Point2f p2, double t2, double target_ts){
    cv::Point2f target_p1, target_p2;
    if ( t1 == target_ts){
        target_p1 = p1;
    }else if (t1 < target_ts){
        auto ptr = time2forwardflow.find(t1);
        target_p1 = p1;
        do{
            cv::Vec3f uv;
            bool valid_value = getValueSubpix<cv::Vec3f>(ptr->second, target_p1, uv);
            if (valid_value){
                target_p1 = target_p1 + cv::Point2f(uv[0], uv[1]);
            }else{
                break;
            }
            ptr++;
        } while(ptr->first < target_ts);
    }else{
        auto fptr = time2backwardflow.find(t1);
        std::map<double, cv::Mat>::reverse_iterator bptr(fptr);
        target_p1 = p1;
        do{
            cv::Vec3f uv;
            bool valid_value = getValueSubpix<cv::Vec3f>(bptr->second, target_p1, uv);
            if (valid_value){
                target_p1 = target_p1 + cv::Point2f(uv[0], uv[1]);
            }else{
                break;
            }
            bptr++;
        } while(bptr->first > target_ts);
    }

    if ( t2 == target_ts){
        target_p2 = p2;
    }else if (t2 < target_ts){
        auto ptr = time2forwardflow.find(t2);
        target_p2 = p2;
        do{
            cv::Vec3f uv;
            bool valid_value = getValueSubpix<cv::Vec3f>(ptr->second, target_p2, uv);
            if (valid_value){
                target_p2 = target_p2 + cv::Point2f(uv[0], uv[1]);
            }else{
                break;
            }
            ptr++;
        } while(ptr->first < target_ts);
    }else{
        auto fptr = time2backwardflow.find(t2);
        std::map<double, cv::Mat>::reverse_iterator bptr(fptr);
        target_p2 = p2;
        do{
            cv::Vec3f uv;
            bool valid_value = getValueSubpix<cv::Vec3f>(bptr->second, target_p2, uv);
            if (valid_value){
                target_p2 = target_p2 + cv::Point2f(uv[0], uv[1]);
            }else{
                break;
            }
            bptr++;
        } while(bptr->first > target_ts);
    }

    return target_p1 - target_p2;
}

std::vector <cv::Point2i> GetBoundaryPixelInward(const cv::Mat &mask) {
//find top left mask pixel
    cv::Mat mask_cpy = mask.clone();
    int cu = -1, cv = -1;
    for (int i = 0; i < mask_cpy.rows; i++) {
        for (int j = 0; j < mask_cpy.cols; j++) {
            if (mask_cpy.at<uchar>(i, j) > 0) {
                cu = j;
                cv = i;
                break;
            }
        }
        if (cv == i)
            break;
    }

    std::vector <cv::Point> dir_offset;
    dir_offset.emplace_back(1, 0);
    dir_offset.emplace_back(0, 1);
    dir_offset.emplace_back(-1, 0);
    dir_offset.emplace_back(0, -1);
    int dir_idx = 0;
    std::vector <cv::Point2i> filling_pix;
    while (mask_cpy.at<uchar>(cv, cu) > 0) {
        bool pix_stored = false;
        if ((dir_idx == 0 && cv > PATCH_SIZE)
            || (dir_idx == 1 && cu + PATCH_SIZE < mask_cpy.cols)
            || (dir_idx == 2 && cv + PATCH_SIZE < mask_cpy.rows)
            || (dir_idx == 3 && cu > PATCH_SIZE)) {
            filling_pix.emplace_back(cu, cv);
            mask_cpy.at<uchar>(cv, cu) = 0;
            pix_stored = true;
        }
        cv::Point2i next_pix = cv::Point2i(cu, cv) + dir_offset[dir_idx];
        if (next_pix.x < 0 || next_pix.x >= mask_cpy.cols || next_pix.y < 0
            || next_pix.y >= mask_cpy.rows
            || mask_cpy.at<uchar>(next_pix.y, next_pix.x) == 0) {
            dir_idx = (dir_idx + 1) % 4;
            if (pix_stored)
                next_pix = cv::Point2i(cu, cv) + dir_offset[dir_idx];
            else
                next_pix = cv::Point2i(cu, cv);
            if (next_pix.x < 0 || next_pix.x >= mask_cpy.cols || next_pix.y < 0
                || next_pix.y >= mask_cpy.rows
                || mask_cpy.at<uchar>(next_pix.y, next_pix.x) == 0)
                break;
        }
        cu = next_pix.x;
        cv = next_pix.y;
    }

    return filling_pix;
}

cv::Rect GetMaskDim(const cv::Mat &mask) {
//find top left mask pixel
    int cu = -1, cv = -1;
    for (int i = 0; i < mask.rows; i++) {
        for (int j = 0; j < mask.cols; j++) {
            if (mask.at<uchar>(i, j) > 0) {
                cu = j;
                cv = i;
                break;
            }
        }
        if (cv == i)
            break;
    }

    int width = 0, height = 0;
    for (int i = cv; i < mask.rows; i++) {
        if (mask.at<uchar>(i, cu) > 0)
            height++;
        else
            break;
    }

    for (int j = cu; j < mask.cols; j++) {
        if (mask.at<uchar>(cv, j) > 0)
            width++;
        else
            break;
    }

    return cv::Rect(cu, cv, width, height);
}

cv::Mat ImageLocalDepth(const std::vector <PointXYZIS> &cloud,
                        const cv::Mat &proj_mat, const cv::Mat &img) {

    cv::Mat dmap(img.size(), CV_32F, cv::Scalar(FLT_MAX));
    cv::Mat z_buffer(img.size(), CV_32F, cv::Scalar(1000000));

    for (auto &p : cloud) {
        cv::Mat v3 = (cv::Mat_<double>(4, 1) << p.x, p.y, p.z, 1);

        cv::Mat uv = proj_mat * v3;
        int u = uv.at<double>(0) / uv.at<double>(2) + 0.5;
        int v = uv.at<double>(1) / uv.at<double>(2) + 0.5;

        if (uv.at<double>(2) > 0 && 0 <= u && u < img.cols && 0 <= v
            && v < img.rows) {
            float p_dist = uv.at<double>(2);

            if (p_dist < z_buffer.at<float>(v, u)) {
                z_buffer.at<float>(v, u) = p_dist;
                dmap.at<float>(v, u) = p_dist;
            }
        }
    }

    cv::Mat_<float> valid_pts(0, 2);
    for (int x = 0; x < dmap.cols; x++) {
        for (int y = 0; y < dmap.rows; y++) {
            if (dmap.at<float>(y, x) < FLT_MAX) {
                cv::Mat row = (cv::Mat_<float>(1, 2) << x, y);
                valid_pts.push_back(row);
            }
        }
    }
    cv::flann::Index flann_index(valid_pts, cv::flann::KDTreeIndexParams(),
                                 cvflann::FLANN_DIST_EUCLIDEAN);

    cv::Mat indices, dists;
    int K = 10;
    for (int x = 0; x < dmap.cols; x++) {
        for (int y = 0; y < dmap.rows; y++) {
            if (dmap.at<float>(y, x) == FLT_MAX) {
                cv::Mat query = (cv::Mat_<float>(1, 2) << x, y);

                flann_index.knnSearch(query, indices, dists, K);

                float ave_d = 0;
                float sum_dist = 0;
                for (int i = 0; i < K; i++) {
                    int ind = indices.at<int>(i);
                    float dist = sqrt(dists.at<float>(i));
                    int sx = valid_pts.at<float>(ind, 0);
                    int sy = valid_pts.at<float>(ind, 1);
                    ave_d += dmap.at<float>(sy, sx) * (1.f / dist);
                    sum_dist += 1.f / dist;
                }

                ave_d /= sum_dist;
                dmap.at<float>(y, x) = ave_d;
            }
        }
    }

    cv::medianBlur(dmap, dmap, 5);

    return dmap;
}

void UpdateInpaintingByDenseSampling(cv::Mat &img, cv::Mat &dmap,
                                     cv::Mat &local_depth, const std::vector <cv::Point2i> &filling_pix,
                                     const std::vector<cv::Mat *> &img_ptrs,
                                     const std::vector<double> &img_scales,
                                     const std::vector <cv::Rect> &img_rois,
                                     const std::vector <cv::Mat> &img_proj_mats, const int vw,
                                     const cv::Mat &K, const cv::Mat &R, const cv::Mat &T,
                                     cv::Rect &filling_rect) {

    cv::Mat color_sum(img.size(), CV_64FC3, cv::Scalar(0, 0, 0));
    cv::Mat color_cnt(img.size(), CV_32S, cv::Scalar(0));
    int cnt = 0;
    cv::Rect img_rect(0, 0, img.cols, img.rows);

    cv::Mat inv_R = R.inv();
    cv::Mat inv_K = K.inv();

    const int ws = 5;

    char outname[512];
    sprintf(outname,
            "../data/pandora_liang/set2/1534313570-1534313581_results/img_input.jpg");
    cv::imwrite(outname, img);

    for (int i = 0; i < filling_pix.size(); i++) {

        cv::Point2i pix = filling_pix[i];

        cv::Mat src_img = *(img_ptrs[i]);

        cv::Rect temp_img_rect(pix.x - PATCH_SIZE, pix.y - PATCH_SIZE,
                               PATCH_SIZE * 2 + 1, PATCH_SIZE * 2 + 1);
        temp_img_rect = temp_img_rect & img_rect;
        cv::Mat temp_img = img(temp_img_rect);
        cv::Mat temp_dmap = dmap(temp_img_rect).clone();
        cv::Mat temp_local_depth = local_depth(temp_img_rect).clone();
        int local_center_x = pix.x - temp_img_rect.x;
        int local_center_y = pix.y - temp_img_rect.y;

        cv::Mat proj_mat = img_proj_mats[i].clone();
        float d = local_depth.at<float>(pix.y, pix.x);
        cv::Mat v3 = (cv::Mat_<double>(3, 1) << pix.x * d, pix.y * d, d);
        cv::Mat p3 = inv_R * (inv_K * v3 - T);
        cv::Mat v4 = (cv::Mat_<double>(4, 1) << p3.at<double>(0), p3.at<double>(
                1), p3.at<double>(2), 1);
        cv::Mat uv = proj_mat * v4;
        float dx = uv.at<double>(0) / uv.at<double>(2), dy = uv.at<double>(1)
                                                             / uv.at<double>(2);

        int dix = dx + 0.5;
        int diy = dy + 0.5;

        if (0 <= dix && dix < src_img.cols && 0 <= diy && diy < src_img.rows) {
            img.at<cv::Vec3b>(pix.y, pix.x) = src_img.at<cv::Vec3b
            >(diy, dix);
        }
    }

    sprintf(outname,
            "../data/pandora_liang/set2/1534313570-1534313581_results/img_output.jpg");
    cv::imwrite(outname, img);
    exit(1);
}

cv::Mat InpaintingByDenseMap(std::vector <PointXYZIS> &cloud, cv::Mat const &R,
                             cv::Mat const &T, cv::Mat const &K, cv::Mat &img, const cv::Mat &mask,
                             cv::Mat &inpainted_init, std::map<double, cv::Mat> &pts2img_proj_mats,
                             double ts, std::map<double, cv::Mat> &time_to_mat,
                             std::map<double, cv::Mat> &mask_time_to_cvmat) {

    cv::Mat PMat;
    cv::hconcat(R, T, PMat);
    PMat = K * PMat;
    cv::Mat full_local_depth = ImageLocalDepth(cloud, PMat, img);
    cv::Mat output_img = img.clone();

    cv::Mat inv_R = R.inv();
    cv::Mat inv_K = K.inv();
    cv::Mat remaining_mask = mask.clone();
    int idx_part = -1;
    std::map<double, cv::Mat>::iterator f_iter = time_to_mat.find(ts);
    std::map<double, cv::Mat>::reverse_iterator b_iter(f_iter);

    while (true) {
        cv::Mat last_inpainted;
        bool looping = true;
        int iter = -1;

        cv::Rect mask_dim = GetMaskDim(remaining_mask);

        if (mask_dim.x == -1) {
            break;
        }

        idx_part++;

        for (int y = mask_dim.y; y < mask_dim.y + mask_dim.height; y++) {
            for (int x = mask_dim.x; x < mask_dim.x + mask_dim.width; x++) {

                float d = full_local_depth.at<float>(y, x);
                cv::Mat v3 = (cv::Mat_<double>(3, 1) << x * d, y * d, d);
                cv::Mat p3 = inv_R * (inv_K * v3 - T);
                cv::Mat v4 = (cv::Mat_<double>(4, 1) << p3.at<double>(0), p3.at<
                        double>(1), p3.at<double>(2), 1);

                auto fi = f_iter;
                auto bi = b_iter;
                do {
                    if (fi != time_to_mat.end())
                        fi++;
                    if (bi != time_to_mat.rend())
                        bi++;

                    if (fi != time_to_mat.end()) {
                        cv::Mat &proj_mat = pts2img_proj_mats[fi->first];
                        cv::Mat &src_mask = mask_time_to_cvmat[fi->first];
                        cv::Mat uv = proj_mat * v4;
                        float dx = uv.at<double>(0) / uv.at<double>(2), dy =
                                uv.at<double>(1) / uv.at<double>(2);

                        int dix = dx + 0.5;
                        int diy = dy + 0.5;

                        if (0 <= dix && dix < fi->second.cols && 0 <= diy
                            && diy < fi->second.rows
                            && src_mask.at<uchar>(diy, dix) == 0) {
                            output_img.at<cv::Vec3b>(y, x) = fi->second.at
                                    <cv::Vec3b>(diy, dix);
                            break;
                        }
                    }

                    if (bi != time_to_mat.rend()) {
                        cv::Mat &proj_mat = pts2img_proj_mats[bi->first];
                        cv::Mat &src_mask = mask_time_to_cvmat[bi->first];
                        cv::Mat uv = proj_mat * v4;
                        float dx = uv.at<double>(0) / uv.at<double>(2), dy =
                                uv.at<double>(1) / uv.at<double>(2);

                        int dix = dx + 0.5;
                        int diy = dy + 0.5;

                        if (0 <= dix && dix < bi->second.cols && 0 <= diy
                            && diy < bi->second.rows
                            && src_mask.at<uchar>(diy, dix) == 0) {
                            output_img.at<cv::Vec3b>(y, x) = bi->second.at
                                    <cv::Vec3b>(diy, dix);
                            break;
                        }
                    }
                } while (fi != time_to_mat.end() || bi != time_to_mat.rend());
            }
        }

        remaining_mask(mask_dim) = 0;
    }

//	sprintf(outname, "../data/pandora_liang/set2/1534313570-1534313581_results/img_output.jpg");
//	cv::imwrite(outname, img);
//	exit(1);
    return output_img;
}



void BPOpt(std::vector <PixLabelColors> &pll, int width, int height,
           int num_label, std::vector<int> &res) {



    std::vector <ValueType> numbersOfLabels(pll.size(), num_label);
    Model gm(SpaceType(numbersOfLabels.begin(), numbersOfLabels.end()));
//    GraphicalModelType gm(SpaceType(numbersOfLabels.begin(), numbersOfLabels.end()));

    // Add functions
    LabelType nl = num_label;
    ExplicitFunction f1(&nl, &nl + 1);

    for (int i = 0; i < pll.size(); ++i) {
        for (int j = 0; j < num_label; j++) {
            f1(j) = pll[i].pix_label_costs[j];
        }
        gm.addFactor(gm.addFunction(f1), &i, &i + 1);
    }

    LabelType l2l[] = {num_label, num_label};
    ExplicitFunction f2(l2l, l2l + 2);
    IndexType vars[] = {0, 1};
    for (IndexType n = 0; n < height; ++n) {
        for (IndexType m = 0; m < width; ++m) {
            vars[0] = n * width + m;
            if (n + 1 < height) { //check for lower neighbor
                vars[1] = (n + 1) * width + m;
                OPENGM_ASSERT(vars[0] < vars[1]); // variables need to be ordered!
                auto &p1 = pll[vars[0]];
                auto &p2 = pll[vars[1]];
                for (LabelType i1 = 0; i1 < num_label; i1++) {
                    for (LabelType i2 = 0; i2 < num_label; i2++) {
                        double cost = cv::norm(p1.bot_colors[i1], p2.pix_colors[i2], CV_L2) * color_smooth_weight +
                                      cv::norm(p1.pix_colors[i1], p2.top_colors[i2], CV_L2) * color_smooth_weight;
                        f2(i1, i2) = cost;
                    }
                }
                gm.addFactor(gm.addFunction(f2), vars, vars + 2);
            }
            if (m + 1 < width) { //check for right neighbor
                vars[1] = n * width + (m + 1);
                OPENGM_ASSERT(vars[0] < vars[1]); // variables need to be ordered!
                auto &p1 = pll[vars[0]];
                auto &p2 = pll[vars[1]];
                for (LabelType i1 = 0; i1 < num_label; i1++) {
                    for (LabelType i2 = 0; i2 < num_label; i2++) {
                        double cost = cv::norm(p1.right_colors[i1], p2.pix_colors[i2], CV_L2) * color_smooth_weight +
                                      cv::norm(p1.pix_colors[i1], p2.left_colors[i2], CV_L2) * color_smooth_weight;
                        f2(i1, i2) = cost;
                    }
                }
                gm.addFactor(gm.addFunction(f2), vars, vars + 2);
            }
        }
    }

    // Inference
    typedef opengm::BeliefPropagationUpdateRules <Model, opengm::Minimizer> UpdateRules;
    typedef opengm::MessagePassing <Model, opengm::Minimizer, UpdateRules,
    opengm::MaxDistance> LBP;
    typedef opengm::ICM <Model, opengm::Minimizer> ICM;
    LBP::Parameter parameter(10, 0.01, 0.8);
    LBP lbp(gm, parameter);
    LBP::VerboseVisitorType visitor;
    lbp.infer(visitor);
    std::vector <LabelType> labeling(gm.numberOfVariables());
    lbp.arg(labeling);

//    MRFLIB::Parameter para;
//    para.inferenceType_ = MRFLIB::Parameter::MAXPRODBP;
//    para.energyType_ = MRFLIB::Parameter::TABLES;
//    para.numberOfIterations_ = 10;
//    MRFLIB mrf(gm,para);
//    mrf.infer();
//    std::vector <LabelType> labeling(gm.numberOfVariables());
//    mrf.arg(labeling);

    res.resize(width * height);
    for (IndexType n = 0; n < height; ++n) {
        for (IndexType m = 0; m < width; ++m) {
            IndexType var = n * width + m;
            res[var] = labeling[var];
        }
    }
}


void BPOptFlow(std::vector <PixLabelColors> &pll, int width, int height,
           int num_label, std::vector<int> &res, double ts) {

    std::vector <ValueType> numbersOfLabels(pll.size(), num_label);
    Model gm(SpaceType(numbersOfLabels.begin(), numbersOfLabels.end()));

    // Add functions
    LabelType nl = num_label;
    ExplicitFunction f1(&nl, &nl + 1);

    for (int i = 0; i < pll.size(); ++i) {
        for (int j = 0; j < num_label; j++) {
            f1(j) = pll[i].pix_label_costs[j];
        }
        gm.addFactor(gm.addFunction(f1), &i, &i + 1);
    }

    LabelType l2l[] = {num_label, num_label};
    ExplicitFunction f2(l2l, l2l + 2);
    IndexType vars[] = {0, 1};
    for (IndexType n = 0; n < height; ++n) {
        for (IndexType m = 0; m < width; ++m) {
            vars[0] = n * width + m;
            if (n + 1 < height) { //check for lower neighbor
                vars[1] = (n + 1) * width + m;
                OPENGM_ASSERT(vars[0] < vars[1]); // variables need to be ordered!
                auto &p1 = pll[vars[0]];
                auto &p2 = pll[vars[1]];
                for (LabelType i1 = 0; i1 < num_label; i1++) {
                    for (LabelType i2 = 0; i2 < num_label; i2++) {
                        cv::Point2f offset = findPixOffset(p1.pix_locs[i1], p1.ts, p2.pix_locs[i2], p2.ts, ts);
                        f2(i1, i2) = cv::norm(offset-cv::Point2f(0, -1)) * color_smooth_weight;
                    }
                }
                gm.addFactor(gm.addFunction(f2), vars, vars + 2);
            }
            if (m + 1 < width) { //check for right neighbor
                vars[1] = n * width + (m + 1);
                OPENGM_ASSERT(vars[0] < vars[1]); // variables need to be ordered!
                auto &p1 = pll[vars[0]];
                auto &p2 = pll[vars[1]];
                for (LabelType i1 = 0; i1 < num_label; i1++) {
                    for (LabelType i2 = 0; i2 < num_label; i2++) {
                        cv::Point2f offset = findPixOffset(p1.pix_locs[i1], p1.ts, p2.pix_locs[i2], p2.ts, ts);
                        f2(i1, i2) = cv::norm(offset-cv::Point2f(-1, 0)) * color_smooth_weight;;
                    }
                }
                gm.addFactor(gm.addFunction(f2), vars, vars + 2);
            }
        }
    }

    // Inference
    typedef opengm::BeliefPropagationUpdateRules <Model, opengm::Minimizer> UpdateRules;
    typedef opengm::MessagePassing <Model, opengm::Minimizer, UpdateRules,
    opengm::MaxDistance> LBP;
    typedef opengm::ICM <Model, opengm::Minimizer> ICM;

    LBP::Parameter parameter(10, 0.01, 0.8);
    LBP lbp(gm, parameter);

    LBP::VerboseVisitorType visitor;
    lbp.infer(visitor);

    std::vector <LabelType> labeling(gm.numberOfVariables());
    lbp.arg(labeling);

    res.resize(width * height);
    for (IndexType n = 0; n < height; ++n) {
        for (IndexType m = 0; m < width; ++m) {
            IndexType var = n * width + m;
            res[var] = labeling[var];
        }
    }
}

bool isBlankPix(cv::Vec3b &c){
    return c[0] == 0 && c[1] ==0 && c[2] ==0;
}

void msg(float s1[VALUES], float s2[VALUES],
         float s3[VALUES], float s4[VALUES],
         float dst[VALUES], std::vector <cv::Vec3b> p_colors, std::vector <cv::Vec3b> q_neigbhor_colors,
         std::vector <cv::Vec3b> p_neighbor_colors, std::vector <cv::Vec3b> q_colors) {

    // aggregate and find min
    float tmp[VALUES];
    for (int value = 0; value < VALUES; value++) {
        tmp[value] = s1[value] + s2[value] + s3[value] + s4[value];
    }

    for (int fq = 0; fq < VALUES; fq++){
        float  min_v = FLT_MAX;
        for (int fp = 0; fp < VALUES; fp++){
            float cdiff = fabs((float)p_colors[fp][0]-q_neigbhor_colors[fq][0]) + fabs((float)p_colors[fp][1]-q_neigbhor_colors[fq][1])+fabs((float)p_colors[fp][2]-q_neigbhor_colors[fq][2]) +
                    fabs((float)p_neighbor_colors[fp][0]-q_colors[fq][0]) + fabs((float)p_neighbor_colors[fp][1]-q_colors[fq][1])+fabs((float)p_neighbor_colors[fp][2]-q_colors[fq][2]);
            float cur_v = tmp[fp] + cdiff * color_smooth_weight;
//            float cur_v = tmp[fp] + cv::norm(p_colors[fp], q_colors[fq], CV_L2) * color_smooth_weight;
            if (cur_v < min_v)
                min_v = cur_v;
        }
        dst[fq] = min_v;
    }

    // normalize
    float val = 0;
    for (int value = 0; value < VALUES; value++)
        val += dst[value];

    val /= VALUES;
    for (int value = 0; value < VALUES; value++)
        dst[value] -= val;
}

void BPOptMsgUpdate(std::vector <PixLabelColors> &pll, int width, int height,
                    std::vector<int> &res){
    const int ITER = 50;

    image<float[VALUES]> *u =new image<float[VALUES]>(width, height);
    image<float[VALUES]> *d =new image<float[VALUES]>(width, height);
    image<float[VALUES]> *l =new image<float[VALUES]>(width, height);
    image<float[VALUES]> *r =new image<float[VALUES]>(width, height);
    image<float[VALUES]> *data =new image<float[VALUES]>(width, height);

    for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; j++) {
            int pix_idx = i * width + j;
            for (int k = 0; k<VALUES; k++)
                imRef(data, j, i)[k] = pll[pix_idx].pix_label_costs[k];
        }
    }

    for (int t = 0; t < ITER; t++) {
//        std::cout << "iter " << t << "\n";
        for (int y = 1; y < height-1; y++) {
            for (int x = ((y+t) % 2) + 1; x < width-1; x+=2) {

                int pix_idx = y * width + x;

//                msg(imRef(u, x, y+1),imRef(l, x+1, y),imRef(r, x-1, y),
//                    imRef(data, x, y), imRef(u, x, y), pll[pix_idx].pix_colors, pll[(y-1)*width+x].bot_colors,
//                    pll[pix_idx].top_colors, pll[(y-1)*width+x].pix_colors);
//
//                msg(imRef(d, x, y-1),imRef(l, x+1, y),imRef(r, x-1, y),
//                    imRef(data, x, y), imRef(d, x, y), pll[pix_idx].pix_colors, pll[(y+1)*width+x].top_colors,
//                    pll[pix_idx].bot_colors, pll[(y+1)*width+x].pix_colors);
//
//                msg(imRef(u, x, y+1),imRef(d, x, y-1),imRef(r, x-1, y),
//                    imRef(data, x, y), imRef(r, x, y), pll[pix_idx].pix_colors, pll[y*width+x+1].left_colors,
//                    pll[pix_idx].right_colors, pll[y*width+x+1].pix_colors);
//
//                msg(imRef(u, x, y+1),imRef(d, x, y-1),imRef(l, x+1, y),
//                    imRef(data, x, y), imRef(l, x, y), pll[pix_idx].pix_colors, pll[y*width+x-1].right_colors,
//                    pll[pix_idx].left_colors, pll[y*width+x-1].pix_colors);

                std::thread t1 = std::thread(msg, imRef(u, x, y+1),imRef(l, x+1, y),imRef(r, x-1, y),
                    imRef(data, x, y), imRef(u, x, y), pll[pix_idx].pix_colors, pll[(y-1)*width+x].bot_colors,
                    pll[pix_idx].top_colors, pll[(y-1)*width+x].pix_colors);

                std::thread t2 = std::thread(msg, imRef(d, x, y-1),imRef(l, x+1, y),imRef(r, x-1, y),
                    imRef(data, x, y), imRef(d, x, y), pll[pix_idx].pix_colors, pll[(y+1)*width+x].top_colors,
                    pll[pix_idx].bot_colors, pll[(y+1)*width+x].pix_colors);

                std::thread t3 = std::thread(msg, imRef(u, x, y+1),imRef(d, x, y-1),imRef(r, x-1, y),
                    imRef(data, x, y), imRef(r, x, y), pll[pix_idx].pix_colors, pll[y*width+x+1].left_colors,
                    pll[pix_idx].right_colors, pll[y*width+x+1].pix_colors);

                std::thread t4 = std::thread(msg, imRef(u, x, y+1),imRef(d, x, y-1),imRef(l, x+1, y),
                    imRef(data, x, y), imRef(l, x, y), pll[pix_idx].pix_colors, pll[y*width+x-1].right_colors,
                    pll[pix_idx].left_colors, pll[y*width+x-1].pix_colors);

                t1.join();
                t2.join();
                t3.join();
                t4.join();

            }
        }
    }

    res.resize(width * height);
    for (int y = 0; y < height; y++){
        for ( int x = 0; x< width; x++){
            auto um=imRef(u, x, y);
            auto dm=imRef(d, x, y);
            auto lm=imRef(l, x, y);
            auto rm=imRef(r, x, y);
            auto c=imRef(data, x, y);

            float min_v = FLT_MAX;
            int min_lb = 0;
            for (int lb = 0; lb < VALUES; lb++){
                float b = um[lb] + dm[lb] + lm[lb] + rm[lb] + c[lb];
                if (b< min_v)
                {
                    min_v = b;
                    min_lb = lb;
                }
            }

            res[y*width+x] = min_lb;
        }
    }

    delete u;
    delete d;
    delete l;
    delete r;
    delete data;
}

cv::Mat InpaintingByDenseMapBP(std::vector <PointXYZIS> &cloud, cv::Mat const &R,
                               cv::Mat const &T, cv::Mat const &K, cv::Mat &img, const cv::Mat &mask,
                               cv::Mat &inpainted_init, std::map<double, cv::Mat> &pts2img_proj_mats,
                               double ts, std::map<double, cv::Mat> &time_to_mat,
                               std::map<double, cv::Mat> &mask_time_to_cvmat) {

    //project pointcloud to current image with z buffer
//    cv::Mat z_buffer(img.size(), CV_32F, cv::Scalar(1000000));
//    cv::Mat pc_proj_img(img.size(), CV_8UC3, cv::Scalar(0, 0, 0));
//    for (auto &p : cloud) {
//        cv::Mat v3 = (cv::Mat_<double>(3, 1) << p.x, p.y, p.z);
//
//        cv::Mat uv = K * (R * v3 + T);
//        int u = uv.at<double>(0) / uv.at<double>(2) + 0.5;
//        int v = uv.at<double>(1) / uv.at<double>(2) + 0.5;
//        float pix_z = uv.at<double>(2);
//        if (pix_z > 0 && 0 <= u && u < pc_proj_img.cols && 0 <= v
//            && v < pc_proj_img.rows && pix_z < z_buffer.at<float>(v, u) && p.img_ptr) {
//            z_buffer.at<float>(v, u) = pix_z;
//            pc_proj_img.at<cv::Vec3b>(v, u) = p.img_ptr->at<cv::Vec3b>(p.v, p.u);
//        }
//    }

    cv::Mat PMat;
    cv::hconcat(R, T, PMat);
    PMat = K * PMat;
    cv::Mat full_local_depth = ImageLocalDepth(cloud, PMat, img);
    cv::Mat output_img = img.clone();

    cv::Mat inv_R = R.inv();
    cv::Mat inv_K = K.inv();
    cv::Mat remaining_mask = mask.clone();
    int idx_part = -1;
    std::map<double, cv::Mat>::iterator f_iter = time_to_mat.find(ts);
    std::map<double, cv::Mat>::reverse_iterator b_iter(f_iter);


    int num_labels = win_size * win_size;
    int half_ws = win_size / 2;

    cv::Mat &cur_img_global_depth = time2depth[ts];

    cv::Rect img_rect(0, 0, img.cols, img.rows);
    while (true) {
        cv::Mat last_inpainted;
        bool looping = true;
        int iter = -1;

        //get next blank region in mask
        cv::Rect mask_dim = GetMaskDim(remaining_mask);

        if (mask_dim.x == -1) {
            break;
        }

        idx_part++;

        std::vector <PixLabelColors> plcs_list;

        for (int y = mask_dim.y; y < mask_dim.y + mask_dim.height; y++) {
            for (int x = mask_dim.x; x < mask_dim.x + mask_dim.width; x++) {
                float d = full_local_depth.at<float>(y, x);
                cv::Mat v3 = (cv::Mat_<double>(3, 1) << x * d, y * d, d);
                cv::Mat p3 = inv_R * (inv_K * v3 - T);
                cv::Mat v4 = (cv::Mat_<double>(4, 1) << p3.at<double>(0), p3.at<
                        double>(1), p3.at<double>(2), 1);

                float global_d = cur_img_global_depth.at<float>(y, x);

                //search forward and backward to find a candidate image with most valid pixels (not overlapping with inpainting area)
                auto fi = f_iter;
                auto bi = b_iter;
                int max_valid_pix_num = 0;
                float min_global_d_diff = FLT_MAX;
                cv::Rect max_rect;
                double max_ts;
                int cx, cy;
                float cfx, cfy;
                do {
                    if (fi != time_to_mat.end())
                        fi++;
                    if (bi != time_to_mat.rend())
                        bi++;

                    if (fi != time_to_mat.end()) {
                        cv::Mat &proj_mat = pts2img_proj_mats[fi->first];
                        cv::Mat &src_mask = mask_time_to_cvmat[fi->first];
                        cv::Mat uv = proj_mat * v4;
                        float dx = uv.at<double>(0) / uv.at<double>(2), dy =
                                uv.at<double>(1) / uv.at<double>(2);

                        int dix = dx + 0.5;
                        int diy = dy + 0.5;

                        if (0 <= dix && dix < fi->second.cols && 0 <= diy
                            && diy < fi->second.rows
                            && src_mask.at<uchar>(diy, dix) == 0) {

                            cv::Rect search_win_fullsize(dix - half_ws,
                                                         diy - half_ws, win_size, win_size);
                            cv::Rect search_win = search_win_fullsize & img_rect;
                            cv::Mat search_win_mask = src_mask(search_win);
                            int num_valid_pix = search_win.area()
                                                - cv::countNonZero(search_win_mask);



                            cv::Mat &targe_img_global_depth = time2depth[fi->first];
                            float target_global_d = targe_img_global_depth.at<float>(diy, dix);
                            float global_d_diff = fabs(target_global_d - global_d);
//                            if (global_d_diff < min_global_d_diff) {
//                                max_valid_pix_num = num_valid_pix;
//                                max_rect = search_win_fullsize;
//                                max_ts = fi->first;
//                                cx = dix;
//                                cy = diy;
//                                cfx = dx;
//                                cfy = dy;
//                                min_global_d_diff = global_d_diff;
//                            }

                            if (num_valid_pix > max_valid_pix_num ) {
                                max_valid_pix_num = num_valid_pix;
                                max_rect = search_win_fullsize;
                                max_ts = fi->first;
                                cx = dix;
                                cy = diy;
                                cfx = dx;
                                cfy = dy;
                                break;
                            }
                        }
                    }

                    if (bi != time_to_mat.rend()) {
                        cv::Mat &proj_mat = pts2img_proj_mats[bi->first];
                        cv::Mat &src_mask = mask_time_to_cvmat[bi->first];
                        cv::Mat uv = proj_mat * v4;
                        float dx = uv.at<double>(0) / uv.at<double>(2), dy =
                                uv.at<double>(1) / uv.at<double>(2);

                        int dix = dx + 0.5;
                        int diy = dy + 0.5;

                        if (0 <= dix && dix < bi->second.cols && 0 <= diy
                            && diy < bi->second.rows
                            && src_mask.at<uchar>(diy, dix) == 0) {
                            cv::Rect search_win_fullsize(dix - half_ws,
                                                         diy - half_ws, win_size, win_size);
                            cv::Rect search_win = search_win_fullsize & img_rect;
                            cv::Mat search_win_mask = src_mask(search_win);
                            int num_valid_pix = search_win.area()
                                                - cv::countNonZero(search_win_mask);

                            cv::Mat &targe_img_global_depth = time2depth[bi->first];
                            float target_global_d = targe_img_global_depth.at<float>(diy, dix);
                            float global_d_diff = fabs(target_global_d - global_d);
//                            if (global_d_diff < min_global_d_diff) {
//                                max_valid_pix_num = num_valid_pix;
//                                max_rect = search_win_fullsize;
//                                max_ts = fi->first;
//                                cx = dix;
//                                cy = diy;
//                                cfx = dx;
//                                cfy = dy;
//                                min_global_d_diff = global_d_diff;
//                            }

                            if (num_valid_pix > max_valid_pix_num) {
                                max_valid_pix_num = num_valid_pix;
                                max_rect = search_win_fullsize;
                                max_ts = bi->first;
                                cx = dix;
                                cy = diy;
                                cfx = dx;
                                cfy = dy;
                                break;
                            }
                        }
                    }

                    if (max_valid_pix_num == win_size * win_size)
                        break;
                } while (fi != time_to_mat.end() || bi != time_to_mat.rend());

//                std::cout<<pts2img_proj_mats.size()<<std::endl;
//                std::cout<<std::to_string(max_ts)<<std::endl;
                cv::Mat &proj_mat = pts2img_proj_mats[max_ts];

                //find neigbhor pix offset in source images
                float left_pix_offset_x, left_pix_offset_y;
                float right_pix_offset_x, right_pix_offset_y;
                float top_pix_offset_x, top_pix_offset_y;
                float bot_pix_offset_x, bot_pix_offset_y;

                float dd;
                if (x > 0)
                    dd = full_local_depth.at<float>(y, x - 1);
                else
                    dd = d;
                v3 = (cv::Mat_<double>(3, 1) << (x - 1) * dd, y * dd, dd);
                p3 = inv_R * (inv_K * v3 - T);
                v4 = (cv::Mat_<double>(4, 1) << p3.at<double>(0), p3.at<double>(
                        1), p3.at<double>(2), 1);

                cv::Mat uv = proj_mat * v4;
                float dx = uv.at<double>(0) / uv.at<double>(2), dy = uv.at<
                        double>(1) / uv.at<double>(2);
                int dix = dx + 0.5, diy = dy + 0.5;
                left_pix_offset_x = dx - cfx;
                left_pix_offset_y = dy - cfy;

                if (x + 1 < full_local_depth.cols)
                    dd = full_local_depth.at<float>(y, x + 1);
                else
                    dd = d;
                v3 = (cv::Mat_<double>(3, 1) << (x + 1) * dd, y * dd, dd);
                p3 = inv_R * (inv_K * v3 - T);
                v4 = (cv::Mat_<double>(4, 1) << p3.at<double>(0), p3.at<double>(
                        1), p3.at<double>(2), 1);
                uv = proj_mat * v4;
                dx = uv.at<double>(0) / uv.at<double>(2);
                dy = uv.at<double>(1) / uv.at<double>(2);
                dix = dx + 0.5;
                diy = dy + 0.5;
                right_pix_offset_x = dx - cfx;
                right_pix_offset_y = dy - cfy;

                if (y > 0)
                    dd = full_local_depth.at<float>(y - 1, x);
                else
                    dd = d;
                v3 = (cv::Mat_<double>(3, 1) << x * dd, (y - 1) * dd, dd);
                p3 = inv_R * (inv_K * v3 - T);
                v4 = (cv::Mat_<double>(4, 1) << p3.at<double>(0), p3.at<double>(
                        1), p3.at<double>(2), 1);
                uv = proj_mat * v4;
                dx = uv.at<double>(0) / uv.at<double>(2);
                dy = uv.at<double>(1) / uv.at<double>(2);
                dix = dx + 0.5;
                diy = dy + 0.5;
                top_pix_offset_x = dx - cfx;
                top_pix_offset_y = dy - cfy;

                if (y + 1 < full_local_depth.rows)
                    dd = full_local_depth.at<float>(y + 1, x);
                else
                    dd = d;
                v3 = (cv::Mat_<double>(3, 1) << x * dd, (y + 1) * dd, dd);
                p3 = inv_R * (inv_K * v3 - T);
                v4 = (cv::Mat_<double>(4, 1) << p3.at<double>(0), p3.at<double>(
                        1), p3.at<double>(2), 1);
                uv = proj_mat * v4;
                dx = uv.at<double>(0) / uv.at<double>(2);
                dy = uv.at<double>(1) / uv.at<double>(2);
                dix = dx + 0.5;
                diy = dy + 0.5;
                bot_pix_offset_x = dx - cfx;
                bot_pix_offset_y = dy - cfy;

                //compute cost of each label color, if there is a projected color,
                //make it similar to that color otherwise, give equal cost to all candidate colors
//                cv::Vec3b proj_color = pc_proj_img.at<cv::Vec3b>(y, x);
//                bool proj_color_exist = false;
//                if (proj_color[0] > 0 || proj_color[1] > 0 || proj_color[2] > 0)
//                    proj_color_exist = true;
                PixLabelColors plcs;
                cv::Mat &src_img = time_to_mat[max_ts];
                cv::Mat &src_mask = mask_time_to_cvmat[max_ts];
                cv::Mat src_img_maskblank = src_img.clone();
                src_img_maskblank.setTo(0, src_mask);
                plcs.ts = max_ts;
                for (int v = max_rect.y; v < max_rect.y + max_rect.height; v++) {
                    for (int u = max_rect.x; u < max_rect.x + max_rect.width; u++) {
                        if (0 <= u && u < src_img.cols && 0 <= v && v < src_img.rows && src_mask.at<uchar>(v, u) == 0) {
                            cv::Vec3b candid_color = src_img.at<cv::Vec3b>(v, u);
                            plcs.pix_colors.push_back(candid_color);
//                            if (proj_color_exist) {
//                                float color_diff = cv::norm(candid_color, proj_color, CV_L2);
//                                plcs.pix_label_costs.push_back(color_diff * proj_color_weight);
//                            } else {
//                                plcs.pix_label_costs.push_back(0);
//                            }
                            float edist = sqrt(float((u-cx) * (u-cx) + (v-cy)*(v-cy)));
//                            if (x%5 ==0 && y%5 == 0) {
                                plcs.pix_label_costs.push_back(edist * center_color_weight);
//                            }
//                            else plcs.pix_label_costs.push_back(0);
                        } else {
                            plcs.pix_colors.push_back(cv::Vec3b(0, 0, 0));
                            plcs.pix_label_costs.push_back(1e10);
                        }
                        plcs.pix_locs.push_back(cv::Point2f(u, v));

                        //store neigbhor's color of current pixel for smoothness const
                        float du = u + left_pix_offset_x;
                        float dv = v + left_pix_offset_y;
                        cv::Vec3b subpix_color(0, 0, 0);
                        getValueSubpix<cv::Vec3b>(src_img_maskblank, cv::Point2f(du, dv), subpix_color);
                        plcs.left_colors.push_back(subpix_color);

                        du = u + right_pix_offset_x;
                        dv = v + right_pix_offset_y;
                        subpix_color=cv::Vec3b(0, 0, 0);
                        getValueSubpix<cv::Vec3b>(src_img_maskblank, cv::Point2f(du, dv), subpix_color);
                        plcs.right_colors.push_back(subpix_color);


                        du = u + top_pix_offset_x;
                        dv = v + top_pix_offset_y;
                        subpix_color=cv::Vec3b(0, 0, 0);
                        getValueSubpix<cv::Vec3b>(src_img_maskblank, cv::Point2f(du, dv), subpix_color);
                        plcs.top_colors.push_back(subpix_color);

                        du = u + bot_pix_offset_x;
                        dv = v + bot_pix_offset_y;
                        subpix_color=cv::Vec3b(0, 0, 0);
                        getValueSubpix<cv::Vec3b>(src_img_maskblank, cv::Point2f(du, dv), subpix_color);
                        plcs.bot_colors.push_back(subpix_color);
//                        std::cout<<left_pix_offset_x<<", "<<left_pix_offset_y<<std::endl;
//                        std::cout<<right_pix_offset_x<<", "<<right_pix_offset_y<<std::endl;
//                        std::cout<<top_pix_offset_x<<", "<<top_pix_offset_y<<std::endl;
//                        std::cout<<bot_pix_offset_x<<", "<<bot_pix_offset_y<<std::endl<<std::endl;

//                        int du = u + left_pix_offset_x;
//                        int dv = v + left_pix_offset_y;
//                        if (0 <= du && du < src_img.cols && 0 <= dv
//                            && dv < src_img.rows
//                            && src_mask.at<uchar>(dv, du) == 0) {
//                            plcs.left_colors.push_back(
//                                    src_img.at<cv::Vec3b>(dv, du));
//                        } else {
//                            plcs.left_colors.push_back(cv::Vec3b(0, 0, 0));
//                        }
//
//                        du = u + right_pix_offset_x;
//                        dv = v + right_pix_offset_y;
//                        if (0 <= du && du < src_img.cols && 0 <= dv
//                            && dv < src_img.rows
//                            && src_mask.at<uchar>(dv, du) == 0) {
//                            plcs.right_colors.push_back(
//                                    src_img.at<cv::Vec3b>(dv, du));
//                        } else {
//                            plcs.right_colors.push_back(cv::Vec3b(0, 0, 0));
//                        }
//
//                        du = u + top_pix_offset_x;
//                        dv = v + top_pix_offset_y;
//                        if (0 <= du && du < src_img.cols && 0 <= dv
//                            && dv < src_img.rows
//                            && src_mask.at<uchar>(dv, du) == 0) {
//                            plcs.top_colors.push_back(
//                                    src_img.at<cv::Vec3b>(dv, du));
//                        } else {
//                            plcs.top_colors.push_back(cv::Vec3b(0, 0, 0));
//                        }
//
//                        du = u + bot_pix_offset_x;
//                        dv = v + bot_pix_offset_y;
//                        if (0 <= du && du < src_img.cols && 0 <= dv
//                            && dv < src_img.rows
//                            && src_mask.at<uchar>(dv, du) == 0) {
//                            plcs.bot_colors.push_back(
//                                    src_img.at<cv::Vec3b>(dv, du));
//                        } else {
//                            plcs.bot_colors.push_back(cv::Vec3b(0, 0, 0));
//                        }
                    }
                }

                //for edge pixels, their neigbhor color should be consistent with boundary
                if (x == mask_dim.x && x > 0) {
                    cv::Vec3b bound_color = img.at<cv::Vec3b>(y, x - 1);
                    for (int l = 0; l < num_labels; l++) {
                        float cdist = cv::norm(plcs.left_colors[l], bound_color, CV_L2);
                        plcs.pix_label_costs[l] += cdist * bound_weight;
                    }
                }

                if (x == mask_dim.x + mask_dim.width - 1 && x < img.cols - 1) {
                    cv::Vec3b bound_color = img.at<cv::Vec3b>(y, x + 1);
                    for (int l = 0; l < num_labels; l++) {
                        float cdist = cv::norm(plcs.right_colors[l], bound_color,
                                               CV_L2);
                        plcs.pix_label_costs[l] += cdist * bound_weight;
                    }
                }

                if (y == mask_dim.y && y > 0) {
                    cv::Vec3b bound_color = img.at<cv::Vec3b>(y - 1, x);
                    for (int l = 0; l < num_labels; l++) {
                        float cdist = cv::norm(plcs.top_colors[l], bound_color,
                                               CV_L2);
                        plcs.pix_label_costs[l] += cdist * bound_weight;
                    }
                }

                if (y == mask_dim.y + mask_dim.height - 1 && y < img.rows - 1) {
                    cv::Vec3b bound_color = img.at<cv::Vec3b>(y + 1, x);
                    for (int l = 0; l < num_labels; l++) {
                        float cdist = cv::norm(plcs.bot_colors[l], bound_color,
                                               CV_L2);
                        plcs.pix_label_costs[l] += cdist * bound_weight;
                    }
                }

                plcs_list.push_back(plcs);
            }
        }

        std::cout<<"start BP..."<<std::endl;
        std::vector<int> res;
//        BPOpt(plcs_list, mask_dim.width, mask_dim.height, win_size * win_size, res);
//        BPOptFlow(plcs_list, mask_dim.width, mask_dim.height, win_size * win_size, res, ts);
        BPOptMsgUpdate(plcs_list, mask_dim.width, mask_dim.height, res);

        int cnt = 0;
        for (int y = mask_dim.y; y < mask_dim.y + mask_dim.height; y++) {
            for (int x = mask_dim.x; x < mask_dim.x + mask_dim.width; x++) {
                int label_idx = res[cnt];
                output_img.at<cv::Vec3b>(y, x) =
                        plcs_list[cnt].pix_colors[label_idx];
                cnt++;
            }
        }

        remaining_mask(mask_dim) = 0;

//        char outname[512];
//        sprintf(outname,
//                "../data/pandora_liang/set2/1534313570-1534313581_results/img_proj.jpg");
//        cv::imwrite(outname, pc_proj_img);
//        sprintf(outname,
//                "../data/pandora_liang/set2/1534313570-1534313581_results/img_input.jpg");
//        cv::imwrite(outname, img);
//        sprintf(outname,
//                "../data/pandora_liang/set2/1534313570-1534313581_results/img_output_csw%f_bw%f_ccw%f_ws%d.jpg",
//                color_smooth_weight, bound_weight, center_color_weight, win_size);
//        cv::imwrite(outname, output_img);
//        exit(1);
    }

    return output_img;
}

cv::Mat InpaintingByDenseMapBP2(std::vector <PointXYZIS> &cloud, cv::Mat const &R,
                               cv::Mat const &T, cv::Mat const &K, cv::Mat &img, const cv::Mat &mask,
                               cv::Mat &inpainted_init, std::map<double, cv::Mat> &pts2img_proj_mats,
                               double ts, std::map<double, cv::Mat> &time_to_mat,
                               std::map<double, cv::Mat> &mask_time_to_cvmat, cv::Mat &pixel_frame_label) {
    cv::Mat PMat;
    cv::hconcat(R, T, PMat);
    PMat = K * PMat;
    cv::Mat full_local_depth = ImageLocalDepth(cloud, PMat, img);
    cv::Mat output_img = img.clone();

    cv::Mat inv_R = R.inv();
    cv::Mat inv_K = K.inv();
    cv::Mat remaining_mask = mask.clone();
    int idx_part = -1;
    std::map<double, cv::Mat>::iterator f_iter = time_to_mat.find(ts);
    std::map<double, cv::Mat>::reverse_iterator b_iter(f_iter);

    double start_time = pts2img_proj_mats.begin()->first;

    int num_labels = win_size * win_size;
    int half_ws = win_size / 2;

    cv::Mat &cur_img_global_depth = time2depth[ts];

    cv::Rect img_rect(0, 0, img.cols, img.rows);
    pixel_frame_label=cv::Mat(img.size(), CV_8U, cv::Scalar(0));
    while (true) {
        cv::Mat last_inpainted;
        bool looping = true;
        int iter = -1;

        //get next blank region in mask
        cv::Rect mask_dim = GetMaskDim(remaining_mask);

        if (mask_dim.x == -1) {
            break;
        }

        idx_part++;

        std::vector <PixLabelColors> plcs_list;

        for (int y = mask_dim.y; y < mask_dim.y + mask_dim.height; y++) {
            for (int x = mask_dim.x; x < mask_dim.x + mask_dim.width; x++) {
                float d = full_local_depth.at<float>(y, x);
                cv::Mat v3 = (cv::Mat_<double>(3, 1) << x * d, y * d, d);
                cv::Mat p3 = inv_R * (inv_K * v3 - T);
                cv::Mat v4 = (cv::Mat_<double>(4, 1) << p3.at<double>(0), p3.at<
                        double>(1), p3.at<double>(2), 1);



                float global_d = cur_img_global_depth.at<float>(y, x);

                //search forward and backward to find a candidate image with most valid pixels (not overlapping with inpainting area)
                auto fi = f_iter;
                auto bi = b_iter;

                float min_global_d_diff = FLT_MAX;
                cv::Rect max_rect;
                double max_ts = 0;
                int cx, cy;
                float cfx, cfy;
                do {
                    if (fi != time_to_mat.end())
                        fi++;

                    if (fi != time_to_mat.end()) {
                        cv::Mat &proj_mat = pts2img_proj_mats[fi->first];
                        cv::Mat &src_mask = mask_time_to_cvmat[fi->first];
                        cv::Mat uv = proj_mat * v4;
                        float dx = uv.at<double>(0) / uv.at<double>(2), dy =
                                uv.at<double>(1) / uv.at<double>(2);

                        int dix = dx + 0.5;
                        int diy = dy + 0.5;

                        if (uv.at<double>(2)>0 && 0 <= dix && dix < fi->second.cols && 0 <= diy
                            && diy < fi->second.rows
                            && src_mask.at<uchar>(diy, dix) == 0) {

                            cv::Rect search_win_fullsize(dix - half_ws,
                                                         diy - half_ws, win_size, win_size);
                            max_rect = search_win_fullsize;
                            max_ts = fi->first;
                            cx = dix;
                            cy = diy;
                            cfx = dx;
                            cfy = dy;
                            break;
                        }
                    }
                } while (fi != time_to_mat.end());

                if (max_ts == 0) {
                    do {
                        if (bi != time_to_mat.rend())
                            bi++;

                        if (bi != time_to_mat.rend()) {
                            cv::Mat &proj_mat = pts2img_proj_mats[bi->first];
                            cv::Mat &src_mask = mask_time_to_cvmat[bi->first];
                            cv::Mat uv = proj_mat * v4;
                            float dx = uv.at<double>(0) / uv.at<double>(2), dy =
                                    uv.at<double>(1) / uv.at<double>(2);

                            int dix = dx + 0.5;
                            int diy = dy + 0.5;

                            if (uv.at<double>(2)>0 && 0 <= dix && dix < bi->second.cols && 0 <= diy
                                && diy < bi->second.rows
                                && src_mask.at<uchar>(diy, dix) == 0) {
                                cv::Rect search_win_fullsize(dix - half_ws,
                                                             diy - half_ws, win_size, win_size);
                                max_rect = search_win_fullsize;
                                max_ts = bi->first;
                                cx = dix;
                                cy = diy;
                                cfx = dx;
                                cfy = dy;
                                break;

                            }
                        }
                    } while (bi != time_to_mat.rend());
                }

                if (max_ts == 0){
                    cx = x;
                    cy = y;
                    cfx = x;
                    cfy = y;
                    cv::Rect search_win_fullsize(x - half_ws,
                                                 y - half_ws, win_size, win_size);
                    max_rect = search_win_fullsize;
                    max_ts = ts;
                }

                pixel_frame_label.at<uchar>(y, x) = (uchar)((max_ts - start_time) *10.0 +0.5);

//                std::cout<<pts2img_proj_mats.size()<<std::endl;
//                std::cout<<std::to_string(max_ts)<<std::endl;
                cv::Mat &proj_mat = pts2img_proj_mats[max_ts];

                //find neigbhor pix offset in source images
                float left_pix_offset_x, left_pix_offset_y;
                float right_pix_offset_x, right_pix_offset_y;
                float top_pix_offset_x, top_pix_offset_y;
                float bot_pix_offset_x, bot_pix_offset_y;

                float dd;
                if (x > 0)
                    dd = full_local_depth.at<float>(y, x - 1);
                else
                    dd = d;
                v3 = (cv::Mat_<double>(3, 1) << (x - 1) * dd, y * dd, dd);
                p3 = inv_R * (inv_K * v3 - T);
                v4 = (cv::Mat_<double>(4, 1) << p3.at<double>(0), p3.at<double>(
                        1), p3.at<double>(2), 1);

                cv::Mat uv = proj_mat * v4;
                float dx = uv.at<double>(0) / uv.at<double>(2), dy = uv.at<
                        double>(1) / uv.at<double>(2);
                int dix = dx + 0.5, diy = dy + 0.5;
                left_pix_offset_x = dx - cfx;
                left_pix_offset_y = dy - cfy;

                if (x + 1 < full_local_depth.cols)
                    dd = full_local_depth.at<float>(y, x + 1);
                else
                    dd = d;
                v3 = (cv::Mat_<double>(3, 1) << (x + 1) * dd, y * dd, dd);
                p3 = inv_R * (inv_K * v3 - T);
                v4 = (cv::Mat_<double>(4, 1) << p3.at<double>(0), p3.at<double>(
                        1), p3.at<double>(2), 1);
                uv = proj_mat * v4;
                dx = uv.at<double>(0) / uv.at<double>(2);
                dy = uv.at<double>(1) / uv.at<double>(2);
                dix = dx + 0.5;
                diy = dy + 0.5;
                right_pix_offset_x = dx - cfx;
                right_pix_offset_y = dy - cfy;

                if (y > 0)
                    dd = full_local_depth.at<float>(y - 1, x);
                else
                    dd = d;
                v3 = (cv::Mat_<double>(3, 1) << x * dd, (y - 1) * dd, dd);
                p3 = inv_R * (inv_K * v3 - T);
                v4 = (cv::Mat_<double>(4, 1) << p3.at<double>(0), p3.at<double>(
                        1), p3.at<double>(2), 1);
                uv = proj_mat * v4;
                dx = uv.at<double>(0) / uv.at<double>(2);
                dy = uv.at<double>(1) / uv.at<double>(2);
                dix = dx + 0.5;
                diy = dy + 0.5;
                top_pix_offset_x = dx - cfx;
                top_pix_offset_y = dy - cfy;

                if (y + 1 < full_local_depth.rows)
                    dd = full_local_depth.at<float>(y + 1, x);
                else
                    dd = d;
                v3 = (cv::Mat_<double>(3, 1) << x * dd, (y + 1) * dd, dd);
                p3 = inv_R * (inv_K * v3 - T);
                v4 = (cv::Mat_<double>(4, 1) << p3.at<double>(0), p3.at<double>(
                        1), p3.at<double>(2), 1);
                uv = proj_mat * v4;
                dx = uv.at<double>(0) / uv.at<double>(2);
                dy = uv.at<double>(1) / uv.at<double>(2);
                dix = dx + 0.5;
                diy = dy + 0.5;
                bot_pix_offset_x = dx - cfx;
                bot_pix_offset_y = dy - cfy;

                PixLabelColors plcs;
                cv::Mat &src_img = time_to_mat[max_ts];
                cv::Mat &src_mask = mask_time_to_cvmat[max_ts];
                cv::Mat src_img_maskblank = src_img.clone();
                src_img_maskblank.setTo(0, src_mask);
                plcs.ts = max_ts;


//                if (x == 1056 && y == 388){
//                    cv::imwrite("../tmp/dst.jpg", img);
//                    cv::imwrite("../tmp/src.jpg", src_img);
//                    std::cout<<cfx<<", "<<cfy<<std::endl;
//
//                    std::cout<<"depth: "<<std::endl;
//                    std::cout<<full_local_depth.at<float>(388, 1056)<<std::endl;
//
//                    cv::Mat output_img;
//                    cv::normalize(full_local_depth, output_img, 0, 255, cv::NORM_MINMAX, CV_8U);
//                    	cv::cvtColor(output_img, output_img, CV_GRAY2BGR);
//                    applyColorMap(output_img, output_img, cv::COLORMAP_JET);
//                    imwrite("../tmp/depth.jpg", output_img);
//                    exit(1);
//                }


                for (int v = max_rect.y; v < max_rect.y + max_rect.height; v++) {
                    for (int u = max_rect.x; u < max_rect.x + max_rect.width; u++) {
                        if (0 <= u && u < src_img.cols && 0 <= v && v < src_img.rows && src_mask.at<uchar>(v, u) == 0) {
                            cv::Vec3b candid_color = src_img.at<cv::Vec3b>(v, u);
                            plcs.pix_colors.push_back(candid_color);

                            float edist = sqrt(float((u-cx) * (u-cx) + (v-cy)*(v-cy)));
                            plcs.pix_label_costs.push_back(edist * center_color_weight);
                        } else {
                            plcs.pix_colors.push_back(cv::Vec3b(0, 0, 0));
                            plcs.pix_label_costs.push_back(1e4);
                        }
                        plcs.pix_locs.push_back(cv::Point2f(u, v));

                        //store neigbhor's color of current pixel for smoothness const
                        float du = u + left_pix_offset_x;
                        float dv = v + left_pix_offset_y;
                        cv::Vec3b subpix_color(0, 0, 0);
                        getValueSubpix<cv::Vec3b>(src_img_maskblank, cv::Point2f(du, dv), subpix_color);
                        plcs.left_colors.push_back(subpix_color);

                        du = u + right_pix_offset_x;
                        dv = v + right_pix_offset_y;
                        subpix_color=cv::Vec3b(0, 0, 0);
                        getValueSubpix<cv::Vec3b>(src_img_maskblank, cv::Point2f(du, dv), subpix_color);
                        plcs.right_colors.push_back(subpix_color);


                        du = u + top_pix_offset_x;
                        dv = v + top_pix_offset_y;
                        subpix_color=cv::Vec3b(0, 0, 0);
                        getValueSubpix<cv::Vec3b>(src_img_maskblank, cv::Point2f(du, dv), subpix_color);
                        plcs.top_colors.push_back(subpix_color);

                        du = u + bot_pix_offset_x;
                        dv = v + bot_pix_offset_y;
                        subpix_color=cv::Vec3b(0, 0, 0);
                        getValueSubpix<cv::Vec3b>(src_img_maskblank, cv::Point2f(du, dv), subpix_color);
                        plcs.bot_colors.push_back(subpix_color);
                    }
                }

                //for edge pixels, their neigbhor color should be consistent with boundary
                if (x == mask_dim.x && x > 0) {
                    cv::Vec3b bound_color = img.at<cv::Vec3b>(y, x - 1);
                    for (int l = 0; l < num_labels; l++) {
                        float cdist = cv::norm(plcs.left_colors[l], bound_color, CV_L2);
                        plcs.pix_label_costs[l] += cdist * bound_weight;
                    }
                }

                if (x == mask_dim.x + mask_dim.width - 1 && x < img.cols - 1) {
                    cv::Vec3b bound_color = img.at<cv::Vec3b>(y, x + 1);
                    for (int l = 0; l < num_labels; l++) {
                        float cdist = cv::norm(plcs.right_colors[l], bound_color,
                                               CV_L2);
                        plcs.pix_label_costs[l] += cdist * bound_weight;
                    }
                }

                if (y == mask_dim.y && y > 0) {
                    cv::Vec3b bound_color = img.at<cv::Vec3b>(y - 1, x);
                    for (int l = 0; l < num_labels; l++) {
                        float cdist = cv::norm(plcs.top_colors[l], bound_color,
                                               CV_L2);
                        plcs.pix_label_costs[l] += cdist * bound_weight;
                    }
                }

                if (y == mask_dim.y + mask_dim.height - 1 && y < img.rows - 1) {
                    cv::Vec3b bound_color = img.at<cv::Vec3b>(y + 1, x);
                    for (int l = 0; l < num_labels; l++) {
                        float cdist = cv::norm(plcs.bot_colors[l], bound_color,
                                               CV_L2);
                        plcs.pix_label_costs[l] += cdist * bound_weight;
                    }
                }

                plcs_list.push_back(plcs);
            }
        }

        std::cout<<"start BP..."<<std::endl;
        std::vector<int> res;
//        BPOpt(plcs_list, mask_dim.width, mask_dim.height, win_size * win_size, res);
//        BPOptFlow(plcs_list, mask_dim.width, mask_dim.height, win_size * win_size, res, ts);
        BPOptMsgUpdate(plcs_list, mask_dim.width, mask_dim.height, res);

        int cnt = 0;
        for (int y = mask_dim.y; y < mask_dim.y + mask_dim.height; y++) {
            for (int x = mask_dim.x; x < mask_dim.x + mask_dim.width; x++) {
                int label_idx = res[cnt];
                output_img.at<cv::Vec3b>(y, x) =
                        plcs_list[cnt].pix_colors[label_idx];
                cnt++;
            }
        }

        remaining_mask(mask_dim) = 0;

//        char outname[512];
//        sprintf(outname,
//                "../data/pandora_liang/set2/1534313570-1534313581_results/img_proj.jpg");
//        cv::imwrite(outname, pc_proj_img);
//        sprintf(outname,
//                "../data/pandora_liang/set2/1534313570-1534313581_results/img_input.jpg");
//        cv::imwrite(outname, img);
//        sprintf(outname,
//                "../data/pandora_liang/set2/1534313570-1534313581_results/img_output_csw%f_bw%f_ccw%f_ws%d.jpg",
//                color_smooth_weight, bound_weight, center_color_weight, win_size);
//        cv::imwrite(outname, output_img);
//        exit(1);
    }

    return output_img;
}

cv::Mat InpaintingByDenseMap2(std::vector <PointXYZIS> &cloud, cv::Mat const &R,
                                cv::Mat const &T, cv::Mat const &K, cv::Mat &img, const cv::Mat &mask,
                                cv::Mat &inpainted_init, std::map<double, cv::Mat> &pts2img_proj_mats,
                                double ts, std::map<double, cv::Mat> &time_to_mat,
                                std::map<double, cv::Mat> &mask_time_to_cvmat, cv::Mat &pixel_frame_label) {
    cv::Mat PMat;
    cv::hconcat(R, T, PMat);
    PMat = K * PMat;
    cv::Mat full_local_depth = ImageLocalDepth(cloud, PMat, img);
    cv::Mat output_img = img.clone();

    cv::Mat inv_R = R.inv();
    cv::Mat inv_K = K.inv();
    cv::Mat remaining_mask = mask.clone();
    int idx_part = -1;
    std::map<double, cv::Mat>::iterator f_iter = time_to_mat.find(ts);
    std::map<double, cv::Mat>::reverse_iterator b_iter(f_iter);

    double start_time = pts2img_proj_mats.begin()->first;

    int num_labels = win_size * win_size;
    int half_ws = win_size / 2;

//    cv::Mat &cur_img_global_depth = time2depth[ts];

    cv::Rect img_rect(0, 0, img.cols, img.rows);
    pixel_frame_label=cv::Mat(img.size(), CV_8U, cv::Scalar(0));
    while (true) {
        cv::Mat last_inpainted;
        bool looping = true;
        int iter = -1;

        //get next blank region in mask
        cv::Rect mask_dim = GetMaskDim(remaining_mask);

        if (mask_dim.x == -1) {
            break;
        }

        idx_part++;

        std::vector <PixLabelColors> plcs_list;

        for (int y = mask_dim.y; y < mask_dim.y + mask_dim.height; y++) {
            for (int x = mask_dim.x; x < mask_dim.x + mask_dim.width; x++) {
                float d = full_local_depth.at<float>(y, x);
                cv::Mat v3 = (cv::Mat_<double>(3, 1) << x * d, y * d, d);
                cv::Mat p3 = inv_R * (inv_K * v3 - T);
                cv::Mat v4 = (cv::Mat_<double>(4, 1) << p3.at<double>(0), p3.at<
                        double>(1), p3.at<double>(2), 1);


//                float global_d = cur_img_global_depth.at<float>(y, x);

                //search forward and backward to find a candidate image with most valid pixels (not overlapping with inpainting area)
                auto fi = f_iter;
                auto bi = b_iter;

                float min_global_d_diff = FLT_MAX;
                cv::Rect max_rect;
                double max_ts = 0;
                int cx, cy;
                float cfx, cfy;
                do {
                    if (fi != time_to_mat.end())
                        fi++;

                    if (fi != time_to_mat.end()) {
                        cv::Mat &proj_mat = pts2img_proj_mats[fi->first];
                        cv::Mat &src_mask = mask_time_to_cvmat[fi->first];
                        cv::Mat uv = proj_mat * v4;
                        float dx = uv.at<double>(0) / uv.at<double>(2), dy =
                                uv.at<double>(1) / uv.at<double>(2);

                        int dix = dx + 0.5;
                        int diy = dy + 0.5;

                        if (uv.at<double>(2) > 0 && 0 <= dix && dix < fi->second.cols && 0 <= diy
                            && diy < fi->second.rows
                            && src_mask.at<uchar>(diy, dix) == 0) {

                            cv::Rect search_win_fullsize(dix - half_ws,
                                                         diy - half_ws, win_size, win_size);
                            max_rect = search_win_fullsize;
                            max_ts = fi->first;
                            cx = dix;
                            cy = diy;
                            cfx = dx;
                            cfy = dy;
                            break;
                        }
                    }
                } while (fi != time_to_mat.end());

                if (max_ts == 0) {
                    do {
                        if (bi != time_to_mat.rend())
                            bi++;

                        if (bi != time_to_mat.rend()) {
                            cv::Mat &proj_mat = pts2img_proj_mats[bi->first];
                            cv::Mat &src_mask = mask_time_to_cvmat[bi->first];
                            cv::Mat uv = proj_mat * v4;
                            float dx = uv.at<double>(0) / uv.at<double>(2), dy =
                                    uv.at<double>(1) / uv.at<double>(2);

                            int dix = dx + 0.5;
                            int diy = dy + 0.5;

                            if (uv.at<double>(2) > 0 && 0 <= dix && dix < bi->second.cols && 0 <= diy
                                && diy < bi->second.rows
                                && src_mask.at<uchar>(diy, dix) == 0) {
                                cv::Rect search_win_fullsize(dix - half_ws,
                                                             diy - half_ws, win_size, win_size);
                                max_rect = search_win_fullsize;
                                max_ts = bi->first;
                                cx = dix;
                                cy = diy;
                                cfx = dx;
                                cfy = dy;
                                break;

                            }
                        }
                    } while (bi != time_to_mat.rend());
                }

                if (max_ts == 0) {
                    cx = x;
                    cy = y;
                    cfx = x;
                    cfy = y;
                    cv::Rect search_win_fullsize(x - half_ws,
                                                 y - half_ws, win_size, win_size);
                    max_rect = search_win_fullsize;
                    max_ts = ts;
                }

                pixel_frame_label.at<uchar>(y, x) = int((max_ts - start_time) * 10.0 + 0.5)%255;
//                pixel_frame_label.at<uchar>(y, x) = int((max_ts - start_time) * 10.0 + 0.5)/4;

                cv::Mat &src_img = time_to_mat[max_ts];
                cv::Mat &src_mask = mask_time_to_cvmat[max_ts];

                output_img.at<cv::Vec3b>(y, x) = src_img.at<cv::Vec3b>(cy, cx);
            }
        }

        remaining_mask(mask_dim) = 0;
    }

    return output_img;
}

cv::Mat InpaintingByDenseMap2BackwardFirst(std::vector <PointXYZIS> &cloud, cv::Mat const &R,
                              cv::Mat const &T, cv::Mat const &K, cv::Mat &img, const cv::Mat &mask,
                              cv::Mat &inpainted_init, std::map<double, cv::Mat> &pts2img_proj_mats,
                              double ts, std::map<double, cv::Mat> &time_to_mat,
                              std::map<double, cv::Mat> &mask_time_to_cvmat, cv::Mat &pixel_frame_label) {
    cv::Mat PMat;
    cv::hconcat(R, T, PMat);
    PMat = K * PMat;
    cv::Mat full_local_depth = ImageLocalDepth(cloud, PMat, img);
    cv::Mat output_img = img.clone();

    cv::Mat inv_R = R.inv();
    cv::Mat inv_K = K.inv();
    cv::Mat remaining_mask = mask.clone();
    int idx_part = -1;
    std::map<double, cv::Mat>::iterator f_iter = time_to_mat.find(ts);
    std::map<double, cv::Mat>::reverse_iterator b_iter(f_iter);

    double start_time = pts2img_proj_mats.begin()->first;

    int num_labels = win_size * win_size;
    int half_ws = win_size / 2;

//    cv::Mat &cur_img_global_depth = time2depth[ts];

    cv::Rect img_rect(0, 0, img.cols, img.rows);
    pixel_frame_label=cv::Mat(img.size(), CV_8U, cv::Scalar(0));
    while (true) {
        cv::Mat last_inpainted;
        bool looping = true;
        int iter = -1;

        //get next blank region in mask
        cv::Rect mask_dim = GetMaskDim(remaining_mask);

        if (mask_dim.x == -1) {
            break;
        }

        idx_part++;

        std::vector <PixLabelColors> plcs_list;

        for (int y = mask_dim.y; y < mask_dim.y + mask_dim.height; y++) {
            for (int x = mask_dim.x; x < mask_dim.x + mask_dim.width; x++) {
                float d = full_local_depth.at<float>(y, x);
                cv::Mat v3 = (cv::Mat_<double>(3, 1) << x * d, y * d, d);
                cv::Mat p3 = inv_R * (inv_K * v3 - T);
                cv::Mat v4 = (cv::Mat_<double>(4, 1) << p3.at<double>(0), p3.at<
                        double>(1), p3.at<double>(2), 1);


//                float global_d = cur_img_global_depth.at<float>(y, x);

                //search forward and backward to find a candidate image with most valid pixels (not overlapping with inpainting area)
                auto fi = f_iter;
                auto bi = b_iter;

                float min_global_d_diff = FLT_MAX;
                cv::Rect max_rect;
                double max_ts = 0;
                int cx, cy;
                float cfx, cfy;

                do {
                    if (bi != time_to_mat.rend())
                        bi++;

                    if (bi != time_to_mat.rend()) {
                        cv::Mat &proj_mat = pts2img_proj_mats[bi->first];
                        cv::Mat &src_mask = mask_time_to_cvmat[bi->first];
                        cv::Mat uv = proj_mat * v4;
                        float dx = uv.at<double>(0) / uv.at<double>(2), dy =
                                uv.at<double>(1) / uv.at<double>(2);

                        int dix = dx + 0.5;
                        int diy = dy + 0.5;

                        if (uv.at<double>(2) > 0 && 0 <= dix && dix < bi->second.cols && 0 <= diy
                            && diy < bi->second.rows
                            && src_mask.at<uchar>(diy, dix) == 0) {
                            cv::Rect search_win_fullsize(dix - half_ws,
                                                         diy - half_ws, win_size, win_size);
                            max_rect = search_win_fullsize;
                            max_ts = bi->first;
                            cx = dix;
                            cy = diy;
                            cfx = dx;
                            cfy = dy;
                            break;

                        }
                    }
                } while (bi != time_to_mat.rend());

                if (max_ts == 0) {


                    do {
                        if (fi != time_to_mat.end())
                            fi++;

                        if (fi != time_to_mat.end()) {
                            cv::Mat &proj_mat = pts2img_proj_mats[fi->first];
                            cv::Mat &src_mask = mask_time_to_cvmat[fi->first];
                            cv::Mat uv = proj_mat * v4;
                            float dx = uv.at<double>(0) / uv.at<double>(2), dy =
                                    uv.at<double>(1) / uv.at<double>(2);

                            int dix = dx + 0.5;
                            int diy = dy + 0.5;

                            if (uv.at<double>(2) > 0 && 0 <= dix && dix < fi->second.cols && 0 <= diy
                                && diy < fi->second.rows
                                && src_mask.at<uchar>(diy, dix) == 0) {

                                cv::Rect search_win_fullsize(dix - half_ws,
                                                             diy - half_ws, win_size, win_size);
                                max_rect = search_win_fullsize;
                                max_ts = fi->first;
                                cx = dix;
                                cy = diy;
                                cfx = dx;
                                cfy = dy;
                                break;
                            }
                        }
                    } while (fi != time_to_mat.end());
                }

                if (max_ts == 0) {
                    cx = x;
                    cy = y;
                    cfx = x;
                    cfy = y;
                    cv::Rect search_win_fullsize(x - half_ws,
                                                 y - half_ws, win_size, win_size);
                    max_rect = search_win_fullsize;
                    max_ts = ts;
                }

                pixel_frame_label.at<uchar>(y, x) = int((max_ts - start_time) * 10.0 + 0.5)%255;


                cv::Mat &src_img = time_to_mat[max_ts];
                cv::Mat &src_mask = mask_time_to_cvmat[max_ts];

                output_img.at<cv::Vec3b>(y, x) = src_img.at<cv::Vec3b>(cy, cx);
            }
        }

        remaining_mask(mask_dim) = 0;
    }

    return output_img;
}

#endif
