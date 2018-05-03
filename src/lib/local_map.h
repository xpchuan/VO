#ifndef VO_MAP_H_
#define VO_MAP_H_

#include <vector>
#include <boost/thread/thread.hpp>  
#include <condition_variable>
#include <map>

#include "matcher.h"
#include "matrix.h"



class KeyFrame{
    public:
        struct parameters {
  
            int32_t nms_n;                  // non-max-suppression: min. distance between maxima (in pixels)
            int32_t nms_tau;                // non-max-suppression: interest point peakiness threshold
            int32_t match_binsize;          // matching bin width/height (affects efficiency only)
            int32_t match_radius;           // matching radius (du/dv in pixels)
            int32_t match_disp_tolerance;   // dv tolerance for stereo matches (in pixels)
            int32_t outlier_disp_tolerance; // outlier removal: disparity tolerance (in pixels)
            int32_t outlier_flow_tolerance; // outlier removal: flow tolerance (in pixels)
            int32_t multi_stage;            // 0=disabled,1=multistage matching (denser and faster)
            int32_t half_resolution;        // 0=disabled,1=match at half resolution, refine at full resolution
            int32_t refinement;             // refinement (0=none,1=pixel,2=subpixel)
            double  f,cu,cv,base;           // calibration (only for match prediction)
            float height, width;
            
            // default settings
            parameters () {
                nms_n                  = 3;
                nms_tau                = 50;
                match_binsize          = 50;
                match_radius           = 200;
                match_disp_tolerance   = 2;
                outlier_disp_tolerance = 5;
                outlier_flow_tolerance = 5;
                multi_stage            = 1;
                half_resolution        = 1;
                refinement             = 1;
            }
        };

        struct MatchedStereo{
            double x;
            double y;
            double z;
            int32_t *lmax;
            int32_t *rmax;
            int count;
        };

        KeyFrame(parameters param, Matrix pose){
            param_ = param;
            pose_ = pose;
        }
        ~KeyFrame(){
            _mm_free (max_des_l_);
            _mm_free (max_des_r_);
        }

        void feedMaximum(int32_t *max_des_l, int32_t num_l,
                         int32_t *max_des_r,  int32_t num_r);
        void stereoMatch();
        // int32_t filtMatches(Matrix pose);

        Matrix pose_;
        int32_t *max_des_l_;
        int32_t *max_des_r_;
        int32_t num_l_;
        int32_t num_r_;
        std::vector<MatchedStereo> stereo_mateched_;
        //std::vector<int32_t> selectes_;

        parameters param_;

    private:
        void createIndexVector (int32_t* m,int32_t n,std::vector<int32_t> *k,
                                const int32_t &u_bin_num,const int32_t &v_bin_num);
        inline void findMatch (int32_t* m1,const int32_t &i1,int32_t* m2,const int32_t &step_size,
                         std::vector<int32_t> *k2,const int32_t &u_bin_num,const int32_t &v_bin_num,const int32_t &stat_bin,
                         int32_t& min_ind,int32_t stage,bool flow,bool use_prior,double u_=-1,double v_=-1);
                         


};

class Map{
    public:
        struct p_match {
            float   u1c,v1c; // u,v-coordinates in current  left  image
            int32_t i1c;     // feature index (for tracking)
            float   u2c,v2c; // u,v-coordinates in current  right image
            int32_t i2c;     // feature index (for tracking)
            double x;
            double y;
            double z;
            p_match(){}
            p_match(float u1c,float v1c,int32_t i1c,float u2c,float v2c,int32_t i2c,
                    double x, double y, double z):
                    u1c(u1c),v1c(v1c),i1c(i1c),u2c(u2c),v2c(v2c),i2c(i2c),
                    x(x), y(y), z(z) {}
        };
        
        // stereo-specific parameters (mandatory: base)
        struct parameters {
            double  base;             // baseline (meters)
            int32_t ransac_iters;     // number of RANSAC iterations
            double  inlier_threshold; // fundamental matrix inlier threshold
            bool    reweighting;      // lower border weights (more robust to calibration errors)
            parameters () {
            base             = 1.0;
            ransac_iters     = 200;
            inlier_threshold = 2.0;
            reweighting      = true;
            }
        };

        Map():map_matched_(false), queue_size_(5){
            ab_pose_ = Matrix::eye(4);
            delta_pose_ = Matrix::eye(4);
            frame_count_ = 0;
            process_th_ = NULL;
        }

        ~Map(){
            for (int i = 0; i < queue_size_; i++){
                delete frames_[i];
            }
        }

        void addFrame(KeyFrame::parameters param,
                    int32_t *max_des_l, int32_t num_l,
                    int32_t *max_des_r,  int32_t mun_r);
        void process();
        Matrix getCorrect(std::vector<double> pre_tr);

        static Map* instance(){
            return instance_;
        }

    private:

        inline void 
        fillFramesIndexVector (std::vector<KeyFrame::MatchedStereo> &couple, Matrix pose,
                                           std::vector<int32_t*> *kl, std::vector<int32_t*> *kr,
                                           const int32_t &u_bin_num,const int32_t &v_bin_num);
        inline void 
        findMatch (int32_t* m1, std::vector<int32_t*> *k2,
                   const int32_t &u_bin_num,const int32_t &v_bin_num,const int32_t &stat_bin,
                   int32_t* &min_ind,int32_t stage,bool flow,bool use_prior,double u_=-1,double v_=-1);

        std::vector<p_match>  p_matched_;
        std::map<int32_t*, int32_t*> couples_record_;
        std::vector<KeyFrame*> frames_;
        KeyFrame::parameters match_param_;

        std::vector<double> estimateMotion (std::vector<p_match> &p_matched, std::vector<double>  pre_tr); 
        enum result { UPDATED, FAILED, CONVERGED };  
        result updateParameters(std::vector<p_match> &p_matched,std::vector<int32_t> &active,std::vector<double> &tr,double step_size,double eps);
        void computeObservations(std::vector<p_match> &p_matched,std::vector<int32_t> &active);
        void computeResidualsAndJacobian(std::vector<double> &tr,std::vector<int32_t> &active);
        std::vector<int32_t> getInlier(std::vector<p_match> &p_matched,std::vector<double> &tr);

        double* X;
        double* Y;
        double* Z;
        double* J;
        double* p_predict;
        double* p_observe;
        double* p_residual;
        Map::parameters es_param_;

        int32_t frame_count_; 
        Matrix  ab_pose_;
        Matrix  delta_pose_;

        //Parameters
        int32_t queue_size_;
        bool map_matched_;

        boost::thread *process_th_;

        static Map* instance_;
};


#endif