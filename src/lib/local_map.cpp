#include <local_map.h>

Map* Map::instance_ = new Map();

std::vector<int32_t> getRandomSample(int32_t N,int32_t num) {

  // init sample and totalset
  std::vector<int32_t> sample;
  std::vector<int32_t> totalset;
  
  totalset.reserve(N);
  sample.reserve(N);
  // create vector containing all indices
  for (int32_t i=0; i<N; i++)
    totalset.push_back(i);

  // add num indices to current sample
  sample.clear();
  for (int32_t i=0; i<num; i++) {
    int32_t j = rand()%totalset.size();
    sample.push_back(totalset[j]);
    totalset.erase(totalset.begin()+j);
  }
  
  // return sample
  return sample;
}

Matrix transformationVectorToMatrix (std::vector<double> tr) {

  // extract parameters
  double rx = tr[0];
  double ry = tr[1];
  double rz = tr[2];
  double tx = tr[3];
  double ty = tr[4];
  double tz = tr[5];

  // precompute sine/cosine
  double sx = sin(rx);
  double cx = cos(rx);
  double sy = sin(ry);
  double cy = cos(ry);
  double sz = sin(rz);
  double cz = cos(rz);

  // compute transformation
  Matrix Tr(4,4);
  Tr.val[0][0] = +cy*cz;          Tr.val[0][1] = -cy*sz;          Tr.val[0][2] = +sy;    Tr.val[0][3] = tx;
  Tr.val[1][0] = +sx*sy*cz+cx*sz; Tr.val[1][1] = -sx*sy*sz+cx*cz; Tr.val[1][2] = -sx*cy; Tr.val[1][3] = ty;
  Tr.val[2][0] = -cx*sy*cz+sx*sz; Tr.val[2][1] = +cx*sy*sz+sx*cz; Tr.val[2][2] = +cx*cy; Tr.val[2][3] = tz;
  Tr.val[3][0] = 0;               Tr.val[3][1] = 0;               Tr.val[3][2] = 0;      Tr.val[3][3] = 1;
  return Tr;
}

// void print_mem(int32_t* input, int32_t num){
//   char* source = (char*)input;
//   for (int i = 0; i < num; i++){
//     char s = *(source + i);
//     printf("%x,", s);
//   }
//   printf("\n");
// }

void KeyFrame::feedMaximum(int32_t *max_des_l, int32_t num_l,
                           int32_t *max_des_r,  int32_t num_r){
    max_des_l_ = (int32_t*)_mm_malloc(sizeof(Matcher::maximum)*num_l,16);
    max_des_r_ = (int32_t*)_mm_malloc(sizeof(Matcher::maximum)*num_r,16);
    memcpy(max_des_l_, max_des_l, num_l * sizeof(Matcher::maximum) * 1);
    memcpy(max_des_r_, max_des_r, num_r * sizeof(Matcher::maximum) * 1);
    num_l_ = num_l;
    num_r_ = num_r;
}

void KeyFrame::stereoMatch(){
  // descriptor step size
  static int32_t step_size = sizeof(Matcher::maximum)/sizeof(int32_t);

  static int32_t u_bin_num = (int32_t)ceil((float)param_.width/(float)param_.match_binsize);
  static int32_t v_bin_num = (int32_t)ceil((float)param_.height/(float)param_.match_binsize);
  static int32_t bin_num   = 4*v_bin_num*u_bin_num;
  

  std::vector<int32_t> *kl = new std::vector<int32_t>[bin_num];
  std::vector<int32_t> *kr = new std::vector<int32_t>[bin_num];

  createIndexVector(max_des_l_,num_l_,kl,u_bin_num,v_bin_num);
  createIndexVector(max_des_r_,num_r_,kr,u_bin_num,v_bin_num);

  bool use_prior = false;
  stereo_mateched_.reserve(1024);

  for (int i1c=0; i1c<num_l_; i1c++) {
  
    int32_t i1c2;
    int32_t i2c;

    // coordinates in previous left image
    int32_t u1c = *(max_des_l_+step_size*i1c+0);
    int32_t v1c = *(max_des_l_+step_size*i1c+1);

    // compute row and column of statistics bin to which this observation belongs
    int32_t u_bin = std::min((int32_t)floor((float)u1c/(float)param_.match_binsize),u_bin_num-1);
    int32_t v_bin = std::min((int32_t)floor((float)v1c/(float)param_.match_binsize),v_bin_num-1);
    int32_t stat_bin = v_bin*u_bin_num+u_bin;

    // match left/right
    findMatch(max_des_l_,i1c,max_des_r_,step_size,kr,u_bin_num,v_bin_num,stat_bin,i2c, 0,false,use_prior);
    findMatch(max_des_r_,i2c,max_des_l_,step_size,kl,u_bin_num,v_bin_num,stat_bin,i1c2,1,false,use_prior);

    // circle closure success?
    if (i1c2==i1c) {

      // extract coordinates
      int32_t u2c = *(max_des_r_+step_size*i2c+0);
      //int32_t v2c = *(max_des_r_+step_size*i2c+1);

      
      // if disparity is positive
      if (u1c>=u2c) {

        MatchedStereo couple;

        couple.lmax = max_des_l_ + step_size*i1c;
        couple.rmax = max_des_r_ + step_size*i2c;

        double d = std::max((double)u1c-(double)u2c,0.0001);
        double x1c = ((double)u1c-param_.cu)*param_.base/d;
        double y1c = ((double)v1c-param_.cv)*param_.base/d;
        double z1c = param_.f*param_.base/d;
        couple.x = x1c;
        couple.y = y1c;
        couple.z = z1c;

        // add match if this pixel isn't matched yet
        stereo_mateched_.push_back(couple);
        
      }
    }
  }

  delete []kl;
  delete []kr;
}

// int32_t KeyFrame::filtMatches(Matrix pose){
//   Matrix project = Matrix::inv(pose) * pose_;

//   double t00,t01,t02,t03,t10,t11,t12,t13,t20,t21,t22,t23;
//   t00 = project.val[0][0];
//   t01 = project.val[0][1];
//   t02 = project.val[0][2];
//   t03 = project.val[0][3];
//   t10 = project.val[1][0];
//   t11 = project.val[1][1];
//   t12 = project.val[1][2];
//   t13 = project.val[1][3];
//   t20 = project.val[2][0];
//   t21 = project.val[2][1];
//   t22 = project.val[2][2];
//   t23 = project.val[2][3];

//   std::vector<int32_t> check_idxes;
//   if (selectes_.empty()){
//     selectes_.reserve(stereo_mateched_.size());
//     for (int i = 0; i < stereo_mateched_.size(); i++)
//       check_idxes.push_back(i);
      
//   }else{
//     check_idxes = selectes_;
//   }
  
//   selectes_.clear();
//   for (int i = 0; i < check_idxes.size(); i++){
//     double x1f = stereo_mateched_[check_idxes[i]].x, 
//            y1f = stereo_mateched_[check_idxes[i]].y,
//            z1f = stereo_mateched_[check_idxes[i]].z;

//     double x1c = t00*x1f + t01*y1f + t02*z1f + t03;
//     double y1c = t10*x1f + t11*y1f + t12*z1f + t13;
//     double z1c = t20*x1f + t21*y1f + t22*z1f + t23;

//     double u1c = param_.f*x1c/z1c+param_.cu;
//     double v1c = param_.f*y1c/z1c+param_.cv;
//     if (u1c < param_.width && u1c > 0 && v1c < param_.height && v1c > 0){
//       selectes_.push_back(check_idxes[i]);
//     }
//   }

//   return selectes_.size();
// }

void KeyFrame::createIndexVector (int32_t* m,int32_t n,std::vector<int32_t> *k,const int32_t &u_bin_num,const int32_t &v_bin_num) {

  // descriptor step size
  int32_t step_size = sizeof(Matcher::maximum)/sizeof(int32_t);
  
  // for all points do
  for (int32_t i=0; i<n; i++) {
    // extract coordinates and class
    int32_t u = *(m+step_size*i+0); // u-coordinate
    int32_t v = *(m+step_size*i+1); // v-coordinate
    int32_t c = *(m+step_size*i+3); // class
    
    // compute row and column of bin to which this observation belongs
    int32_t u_bin = std::min((int32_t)floor((float)u/(float)param_.match_binsize),u_bin_num-1);
    int32_t v_bin = std::min((int32_t)floor((float)v/(float)param_.match_binsize),v_bin_num-1);
    
    // save index
    k[(c*v_bin_num+v_bin)*u_bin_num+u_bin].push_back(i);
  }
}

inline void KeyFrame::findMatch (int32_t* m1,const int32_t &i1,int32_t* m2,const int32_t &step_size,std::vector<int32_t> *k2,
                                const int32_t &u_bin_num,const int32_t &v_bin_num,const int32_t &stat_bin,
                                int32_t& min_ind,int32_t stage,bool flow,bool use_prior,double u_,double v_) {
  
  // init and load image coordinates + feature
  min_ind          = 0;
  double  min_cost = 10000000;
  int32_t u1       = *(m1+step_size*i1+0);
  int32_t v1       = *(m1+step_size*i1+1);
  int32_t c        = *(m1+step_size*i1+3);
  __m128i xmm1     = _mm_load_si128((__m128i*)(m1+step_size*i1+4));
  __m128i xmm2     = _mm_load_si128((__m128i*)(m1+step_size*i1+8));
  
  float u_min,u_max,v_min,v_max;
  
  // restrict search range with prior
  if (0) {
    // u_min = u1+ranges[stat_bin].u_min[stage];
    // u_max = u1+ranges[stat_bin].u_max[stage];
    // v_min = v1+ranges[stat_bin].v_min[stage];
    // v_max = v1+ranges[stat_bin].v_max[stage];
    return;
    
  // otherwise: use full search space
  } else {
    u_min = u1-param_.match_radius;
    u_max = u1+param_.match_radius;
    v_min = v1-param_.match_radius;
    v_max = v1+param_.match_radius;
  }
  
  // if stereo search => constrain to 1d
  if (!flow) {
    v_min = v1-param_.match_disp_tolerance;
    v_max = v1+param_.match_disp_tolerance;
  }
  
  // bins of interest
  int32_t u_bin_min = std::min(std::max((int32_t)floor(u_min/(float)param_.match_binsize),0),u_bin_num-1);
  int32_t u_bin_max = std::min(std::max((int32_t)floor(u_max/(float)param_.match_binsize),0),u_bin_num-1);
  int32_t v_bin_min = std::min(std::max((int32_t)floor(v_min/(float)param_.match_binsize),0),v_bin_num-1);
  int32_t v_bin_max = std::min(std::max((int32_t)floor(v_max/(float)param_.match_binsize),0),v_bin_num-1);
  
  // for all bins of interest do
  for (int32_t u_bin=u_bin_min; u_bin<=u_bin_max; u_bin++) {
    for (int32_t v_bin=v_bin_min; v_bin<=v_bin_max; v_bin++) {
      int32_t k2_ind = (c*v_bin_num+v_bin)*u_bin_num+u_bin;
      for (std::vector<int32_t>::const_iterator i2_it=k2[k2_ind].begin(); i2_it!=k2[k2_ind].end(); i2_it++) {
        int32_t u2   = *(m2+step_size*(*i2_it)+0);
        int32_t v2   = *(m2+step_size*(*i2_it)+1);
        if (u2>=u_min && u2<=u_max && v2>=v_min && v2<=v_max) {
          __m128i xmm3 = _mm_load_si128((__m128i*)(m2+step_size*(*i2_it)+4));
          __m128i xmm4 = _mm_load_si128((__m128i*)(m2+step_size*(*i2_it)+8));                    
          xmm3 = _mm_sad_epu8 (xmm1,xmm3);
          xmm4 = _mm_sad_epu8 (xmm2,xmm4);
          xmm4 = _mm_add_epi16(xmm3,xmm4);
          double cost = (double)(_mm_extract_epi16(xmm4,0)+_mm_extract_epi16(xmm4,4));
          
          if (u_>=0 && v_>=0) {
            double du = (double)u2-u_;
            double dv = (double)v2-v_;
            double dist = sqrt(du*du+dv*dv);
            cost += 4*dist;
          }
          
          if (cost<min_cost) {
            min_ind  = *i2_it;
            min_cost = cost;
          }
        }
      }
    }
  }
}

void Map::addFrame(KeyFrame::parameters param,
                    int32_t *max_des_l, int32_t num_l,
                    int32_t *max_des_r,  int32_t num_r){
  frame_count_ ++;
  printf("+++ start add frame +++\n");
  if (frames_.size() == queue_size_){
    delete frames_.back();
    frames_.pop_back();
  }

  printf("+++ add frame +++\n");
  if (frames_.size() > queue_size_)
    std::cerr << "Map : Frame Queue Error";

  Matrix predict = ab_pose_ * Matrix::inv(delta_pose_);
  KeyFrame* frame = new KeyFrame(param, predict);
  frame->feedMaximum(max_des_l, num_l, max_des_r, num_r);
  
  frames_.insert(frames_.begin(), frame);
  match_param_ = param;
  
  printf("create thread\n");
  process_th_ = new std::thread(&Map::process, this);
}

inline void Map::fillFramesIndexVector(std::vector<KeyFrame::MatchedStereo> &couple, Matrix project,
                                       std::vector<int32_t*> *kl, std::vector<int32_t*> *kr,
                                       const int32_t &u_bin_num,const int32_t &v_bin_num){
  
  printf("+++ fill frame +++\n");
  // descriptor step size
  int32_t step_size = sizeof(Matcher::maximum)/sizeof(int32_t);
  
  int32_t n = couple.size();

  double t00,t01,t02,t03,t10,t11,t12,t13,t20,t21,t22,t23;
  t00 = project.val[0][0];
  t01 = project.val[0][1];
  t02 = project.val[0][2];
  t03 = project.val[0][3];
  t10 = project.val[1][0];
  t11 = project.val[1][1];
  t12 = project.val[1][2];
  t13 = project.val[1][3];
  t20 = project.val[2][0];
  t21 = project.val[2][1];
  t22 = project.val[2][2];
  t23 = project.val[2][3];

  couples_record_.clear();

  printf("+++ couples record +++\n");
  // for all points do
  for (int32_t i=0; i<n; i++) {
    

    double x1f = couple[i].x;
    double y1f = couple[i].y;
    double z1f = couple[i].z;

    int32_t *lmax = couple[i].lmax;
    int32_t *rmax = couple[i].rmax;


    double x1c = t00*x1f + t01*y1f + t02*z1f + t03;
    double y1c = t10*x1f + t11*y1f + t12*z1f + t13;
    double z1c = t20*x1f + t21*y1f + t22*z1f + t23;

    int32_t u1c = static_cast<int32_t>(match_param_.f*x1c/z1c+match_param_.cu);
    int32_t v1c = static_cast<int32_t>(match_param_.f*y1c/z1c+match_param_.cv);
    int32_t c1c = *(lmax+3); // class

    double x2c = x1c - match_param_.base;

    int32_t u2c = static_cast<int32_t>(match_param_.f*x2c/z1c+match_param_.cu);
    int32_t v2c = static_cast<int32_t>(match_param_.f*y1c/z1c+match_param_.cv);
    int32_t c2c = *(rmax+3); // class
    
    if (u1c > match_param_.width || u1c < 0 || v1c > match_param_.height || v1c < 0 ||
        u2c > match_param_.width || u2c < 0 || v2c > match_param_.height || v2c < 0 )
        continue;

    // compute row and column of bin to which this observation belongs
    int32_t u1_bin = std::min((int32_t)floor((float)u1c/(float)match_param_.match_binsize),u_bin_num-1);
    int32_t v1_bin = std::min((int32_t)floor((float)v1c/(float)match_param_.match_binsize),v_bin_num-1);
    int32_t u2_bin = std::min((int32_t)floor((float)u2c/(float)match_param_.match_binsize),u_bin_num-1);
    int32_t v2_bin = std::min((int32_t)floor((float)v2c/(float)match_param_.match_binsize),v_bin_num-1);
    
    // save index
    kl[(c1c*v_bin_num+v1_bin)*u_bin_num+u1_bin].push_back(lmax);
    kr[(c2c*v_bin_num+v2_bin)*u_bin_num+u2_bin].push_back(rmax);

    couples_record_[lmax] = rmax;
  }
  printf("+++ finish record +++\n");
}

inline void Map::findMatch (int32_t* m1,std::vector<int32_t*> *k2,
                                const int32_t &u_bin_num,const int32_t &v_bin_num,const int32_t &stat_bin,
                                int32_t* &min_ind,int32_t stage,bool flow,bool use_prior,double u_,double v_) {
  
  // init and load image coordinates + feature
  min_ind          = 0;
  double  min_cost = 10000000;
  int32_t u1       = *(m1+0);
  int32_t v1       = *(m1+1);
  int32_t c        = *(m1+3);
  __m128i xmm1     = _mm_load_si128((__m128i*)(m1+4));
  __m128i xmm2     = _mm_load_si128((__m128i*)(m1+8));
  
  float u_min,u_max,v_min,v_max;
  
  // restrict search range with prior
  if (0) {
    // u_min = u1+ranges[stat_bin].u_min[stage];
    // u_max = u1+ranges[stat_bin].u_max[stage];
    // v_min = v1+ranges[stat_bin].v_min[stage];
    // v_max = v1+ranges[stat_bin].v_max[stage];
    return;
    
  // otherwise: use full search space
  } else {
    u_min = u1-match_param_.match_radius;
    u_max = u1+match_param_.match_radius;
    v_min = v1-match_param_.match_radius;
    v_max = v1+match_param_.match_radius;
  }
  
  // if stereo search => constrain to 1d
  if (!flow) {
    v_min = v1-match_param_.match_disp_tolerance;
    v_max = v1+match_param_.match_disp_tolerance;
  }
  
  // bins of interest
  int32_t u_bin_min = std::min(std::max((int32_t)floor(u_min/(float)match_param_.match_binsize),0),u_bin_num-1);
  int32_t u_bin_max = std::min(std::max((int32_t)floor(u_max/(float)match_param_.match_binsize),0),u_bin_num-1);
  int32_t v_bin_min = std::min(std::max((int32_t)floor(v_min/(float)match_param_.match_binsize),0),v_bin_num-1);
  int32_t v_bin_max = std::min(std::max((int32_t)floor(v_max/(float)match_param_.match_binsize),0),v_bin_num-1);
  
  // for all bins of interest do
  for (int32_t u_bin=u_bin_min; u_bin<=u_bin_max; u_bin++) {
    for (int32_t v_bin=v_bin_min; v_bin<=v_bin_max; v_bin++) {
      int32_t k2_ind = (c*v_bin_num+v_bin)*u_bin_num+u_bin;
      for (std::vector<int32_t*>::const_iterator i2_it=k2[k2_ind].begin(); i2_it!=k2[k2_ind].end(); i2_it++) {
        int32_t u2   = *(*i2_it+0);
        int32_t v2   = *(*i2_it+1);
        if (u2>=u_min && u2<=u_max && v2>=v_min && v2<=v_max) {
          __m128i xmm3 = _mm_load_si128((__m128i*)(*i2_it+4));
          __m128i xmm4 = _mm_load_si128((__m128i*)(*i2_it+8));                    
          xmm3 = _mm_sad_epu8 (xmm1,xmm3);
          xmm4 = _mm_sad_epu8 (xmm2,xmm4);
          xmm4 = _mm_add_epi16(xmm3,xmm4);
          double cost = (double)(_mm_extract_epi16(xmm4,0)+_mm_extract_epi16(xmm4,4));
          
          if (u_>=0 && v_>=0) {
            double du = (double)u2-u_;
            double dv = (double)v2-v_;
            double dist = sqrt(du*du+dv*dv);
            cost += 4*dist;
          }
          
          if (cost<min_cost) {
            min_ind  = *i2_it;
            min_cost = cost;
          }
        }
      }
    }
  }
}

void Map::process(){
  bool use_prior = false;

  frames_[0]->stereoMatch();

  printf("process start\n");
  // descriptor step size
  int32_t step_size = sizeof(Matcher::maximum)/sizeof(int32_t);
  int32_t u_bin_num = (int32_t)ceil((float)match_param_.width/(float)match_param_.match_binsize);
  int32_t v_bin_num = (int32_t)ceil((float)match_param_.height/(float)match_param_.match_binsize);
  int32_t bin_num   = 4*v_bin_num*u_bin_num;

  std::vector<int32_t*> *klf = new std::vector<int32_t*>[bin_num];
  std::vector<int32_t*> *krf = new std::vector<int32_t*>[bin_num];

  if (frames_.size() > 1) {
    for(int i = 1; i < frames_.size(); i++) {
      Matrix project = Matrix::inv(frames_[0]->pose_) * frames_[i]->pose_;
      fillFramesIndexVector(frames_[i]->stereo_mateched_, project, klf, krf, u_bin_num, v_bin_num);
    }
  }

  printf("filled finish %d\n", frames_.size());

  if (frames_.size() >= queue_size_){
    printf("start-match---->>>>>>>%d\n", frames_[0]->stereo_mateched_.size());
    int32_t n1c = frames_[0]->num_l_;
    int32_t n2c = frames_[0]->num_r_;
    int32_t *m1c = frames_[0]->max_des_l_;
    int32_t *m2c = frames_[0]->max_des_r_;

    for (auto &select:frames_[0]->stereo_mateched_){
      int32_t* l1cf;
      int32_t* r1cf;

      int32_t *lmax = select.lmax;
      int32_t *rmax = select.rmax;

      // coordinates in previous left image
      int32_t u1c = *(lmax+0);
      int32_t v1c = *(lmax+1);
      int32_t i1c = *(lmax+2);

      int32_t u2c = *(rmax+0);
      int32_t v2c = *(rmax+1);
      int32_t i2c = *(rmax+2);

      // compute row and column of statistics bin to which this observation belongs
      int32_t u1_bin = std::min((int32_t)floor((float)u1c/(float)match_param_.match_binsize),u_bin_num-1);
      int32_t v1_bin = std::min((int32_t)floor((float)v1c/(float)match_param_.match_binsize),v_bin_num-1);
      int32_t stat1_bin = v1_bin*u_bin_num+u1_bin;

      int32_t u2_bin = std::min((int32_t)floor((float)u1c/(float)match_param_.match_binsize),u_bin_num-1);
      int32_t v2_bin = std::min((int32_t)floor((float)v1c/(float)match_param_.match_binsize),v_bin_num-1);
      int32_t stat2_bin = v2_bin*u_bin_num+u2_bin;

      findMatch(lmax,klf,u_bin_num,v_bin_num,stat1_bin,l1cf, 1,true ,use_prior);
      findMatch(rmax,krf,u_bin_num,v_bin_num,stat2_bin,r1cf, 1,true ,use_prior);

      if (couples_record_[l1cf] == r1cf){
        p_match match;
        match.i1c = i1c;
        match.i2c = i2c;
        match.u1c = u1c;
        match.u2c = u2c;
        match.v1c = v1c;
        match.v2c = v2c;
        match.x = select.x;
        match.y = select.y;
        match.z = select.z;
        p_matched_.push_back(match);
      }
    }
    map_matched_ = true;
  }else{
    map_matched_ = false;
  }

  printf("process finish\n");
}

Matrix Map::getCorrect(std::vector<double> pre_tr){

  printf("wait for process join\n");
  if (process_th_){
    process_th_->join();
    delete process_th_;
    process_th_ = NULL;
  }

  std::vector<double> delta;
  if (!map_matched_){
    delta = pre_tr;
  }else{
    std::vector<double>  tr_delta = estimateMotion(p_matched_, pre_tr);
    if (tr_delta.size() != 6)
      delta = pre_tr;
    else
      delta = tr_delta;
  }

  if (delta.size() != 6)
    delta = std::vector<double>(6, 0.);
  
  delta_pose_ = transformationVectorToMatrix(delta);
  ab_pose_ = ab_pose_ * Matrix::inv(delta_pose_);
  if (!frames_.empty())
    frames_[0]->pose_ = ab_pose_;
  return ab_pose_;
}

std::vector<double> 
Map::estimateMotion (std::vector<Map::p_match> &p_matched, std::vector<double>  pre_tr) {
  
  // return value
  bool success = true;
  
  // compute minimum distance for RANSAC samples
  double width=0,height=0;
  for (std::vector<p_match>::iterator it=p_matched.begin(); it!=p_matched.end(); it++) {
    if (it->u1c>width)  width  = it->u1c;
    if (it->v1c>height) height = it->v1c;
  }
  double min_dist = std::min(width,height)/3.0;
  
  // get number of matches
  int32_t N  = p_matched.size();
  if (N<6)
    return std::vector<double>();

  // allocate dynamic memory
  X          = new double[N];
  Y          = new double[N];
  Z          = new double[N];
  J          = new double[4*N*6];
  p_predict  = new double[4*N];
  p_observe  = new double[4*N];
  p_residual = new double[4*N];

  // project matches of previous image into 3d
  for (int32_t i=0; i<N; i++) {
    X[i] = p_matched[i].x;
    Y[i] = p_matched[i].y;
    Z[i] = p_matched[i].z;
  }

  // loop variables
  std::vector<double> tr_delta;
  std::vector<double> tr_delta_curr;
  tr_delta_curr.resize(6);
  
  // clear parameter vector
  std::vector<int32_t> inliers;

  // initial RANSAC estimate
  for (int32_t k=0;k<es_param_.ransac_iters;k++) {

    // draw random sample set
    std::vector<int32_t> active = getRandomSample(N,3);

    // clear parameter vector
    // for (int32_t i=0; i<6; i++)
    //   tr_delta_curr[i] = 0;
    tr_delta_curr = pre_tr;

    // minimize reprojection errors
    result result = UPDATED;
    int32_t iter=0;
    while (result==UPDATED) {
      result = updateParameters(p_matched,active,tr_delta_curr,1,1e-6);
      if (iter++ > 20 || result==CONVERGED)
        break;
    }

    // overwrite best parameters if we have more inliers
    if (result!=FAILED) {
      std::vector<int32_t> inliers_curr = getInlier(p_matched,tr_delta_curr);
      if (inliers_curr.size()>inliers.size()) {
        inliers = inliers_curr;
        tr_delta = tr_delta_curr;
      }
    }
  }
  
  // final optimization (refinement)
  if (inliers.size()>=6) {
    int32_t iter=0;
    result result = UPDATED;
    while (result==UPDATED) {     
      result = updateParameters(p_matched,inliers,tr_delta,1,1e-8);
      if (iter++ > 100 || result==CONVERGED)
        break;
    }

    // not converged
    if (result!=CONVERGED)
      success = false;

  // not enough inliers
  } else {
    success = false;
  }

  // release dynamic memory
  delete[] X;
  delete[] Y;
  delete[] Z;
  delete[] J;
  delete[] p_predict;
  delete[] p_observe;
  delete[] p_residual;
  
  // parameter estimate succeeded?
  if (success) return tr_delta;
  else         return std::vector<double>();
}

std::vector<int32_t> Map::getInlier(std::vector<Map::p_match> &p_matched,std::vector<double> &tr) {

  // mark all observations active
  std::vector<int32_t> active;
  for (int32_t i=0; i<(int32_t)p_matched.size(); i++)
    active.push_back(i);

  // extract observations and compute predictions
  computeObservations(p_matched,active);
  computeResidualsAndJacobian(tr,active);

  // compute inliers
  std::vector<int32_t> inliers;
  for (int32_t i=0; i<(int32_t)p_matched.size(); i++)
    if (pow(p_observe[4*i+0]-p_predict[4*i+0],2)+pow(p_observe[4*i+1]-p_predict[4*i+1],2) +
        pow(p_observe[4*i+2]-p_predict[4*i+2],2)+pow(p_observe[4*i+3]-p_predict[4*i+3],2) < es_param_.inlier_threshold*es_param_.inlier_threshold)
      inliers.push_back(i);
  return inliers;
}

Map::result Map::updateParameters(std::vector<Map::p_match> &p_matched,std::vector<int32_t> &active,std::vector<double> &tr,double step_size,double eps) {
  
  // we need at least 3 observations
  if (active.size()<3)
    return FAILED;
  
  // extract observations and compute predictions
  computeObservations(p_matched,active);
  computeResidualsAndJacobian(tr,active);

  // init
  Matrix A(6,6);
  Matrix B(6,1);

  // fill matrices A and B
  for (int32_t m=0; m<6; m++) {
    for (int32_t n=0; n<6; n++) {
      double a = 0;
      for (int32_t i=0; i<4*(int32_t)active.size(); i++) {
        a += J[i*6+m]*J[i*6+n];
      }
      A.val[m][n] = a;
    }
    double b = 0;
    for (int32_t i=0; i<4*(int32_t)active.size(); i++) {
      b += J[i*6+m]*(p_residual[i]);
    }
    B.val[m][0] = b;
  }

  // perform elimination
  if (B.solve(A)) {
    bool converged = true;
    for (int32_t m=0; m<6; m++) {
      tr[m] += step_size*B.val[m][0];
      if (fabs(B.val[m][0])>eps)
        converged = false;
    }
    if (converged)
      return CONVERGED;
    else
      return UPDATED;
  } else {
    return FAILED;
  }
}

void Map::computeObservations(std::vector<p_match> &p_matched,std::vector<int32_t> &active) {

  // set all observations
  for (int32_t i=0; i<(int32_t)active.size(); i++) {
    p_observe[4*i+0] = p_matched[active[i]].u1c; // u1
    p_observe[4*i+1] = p_matched[active[i]].v1c; // v1
    p_observe[4*i+2] = p_matched[active[i]].u2c; // u2
    p_observe[4*i+3] = p_matched[active[i]].v2c; // v2
  }
}

void Map::computeResidualsAndJacobian(std::vector<double> &tr,std::vector<int32_t> &active) {

  // extract motion parameters
  double rx = tr[0]; double ry = tr[1]; double rz = tr[2];
  double tx = tr[3]; double ty = tr[4]; double tz = tr[5];

  // precompute sine/cosine
  double sx = sin(rx); double cx = cos(rx); double sy = sin(ry);
  double cy = cos(ry); double sz = sin(rz); double cz = cos(rz);

  // compute rotation matrix and derivatives
  double r00    = +cy*cz;          double r01    = -cy*sz;          double r02    = +sy;
  double r10    = +sx*sy*cz+cx*sz; double r11    = -sx*sy*sz+cx*cz; double r12    = -sx*cy;
  double r20    = -cx*sy*cz+sx*sz; double r21    = +cx*sy*sz+sx*cz; double r22    = +cx*cy;
  double rdrx10 = +cx*sy*cz-sx*sz; double rdrx11 = -cx*sy*sz-sx*cz; double rdrx12 = -cx*cy;
  double rdrx20 = +sx*sy*cz+cx*sz; double rdrx21 = -sx*sy*sz+cx*cz; double rdrx22 = -sx*cy;
  double rdry00 = -sy*cz;          double rdry01 = +sy*sz;          double rdry02 = +cy;
  double rdry10 = +sx*cy*cz;       double rdry11 = -sx*cy*sz;       double rdry12 = +sx*sy;
  double rdry20 = -cx*cy*cz;       double rdry21 = +cx*cy*sz;       double rdry22 = -cx*sy;
  double rdrz00 = -cy*sz;          double rdrz01 = -cy*cz;
  double rdrz10 = -sx*sy*sz+cx*cz; double rdrz11 = -sx*sy*cz-cx*sz;
  double rdrz20 = +cx*sy*sz+sx*cz; double rdrz21 = +cx*sy*cz-sx*sz;

  // loop variables
  double X1p,Y1p,Z1p;
  double X1c,Y1c,Z1c,X2c;
  double X1cd,Y1cd,Z1cd;

  // for all observations do
  for (int32_t i=0; i<(int32_t)active.size(); i++) {

    // get 3d point in previous coordinate system
    X1p = X[active[i]];
    Y1p = Y[active[i]];
    Z1p = Z[active[i]];

    // compute 3d point in current left coordinate system
    X1c = r00*X1p+r01*Y1p+r02*Z1p+tx;
    Y1c = r10*X1p+r11*Y1p+r12*Z1p+ty;
    Z1c = r20*X1p+r21*Y1p+r22*Z1p+tz;
    
    // weighting
    double weight = 1.0;
    if (es_param_.reweighting)
      weight = 1.0/(fabs(p_observe[4*i+0]-match_param_.cu)/fabs(match_param_.cu) + 0.05);
    
    // compute 3d point in current right coordinate system
    X2c = X1c-match_param_.base;

    // for all paramters do
    for (int32_t j=0; j<6; j++) {

      // derivatives of 3d pt. in curr. left coordinates wrt. param j
      switch (j) {
        case 0: X1cd = 0;
                Y1cd = rdrx10*X1p+rdrx11*Y1p+rdrx12*Z1p;
                Z1cd = rdrx20*X1p+rdrx21*Y1p+rdrx22*Z1p;
                break;
        case 1: X1cd = rdry00*X1p+rdry01*Y1p+rdry02*Z1p;
                Y1cd = rdry10*X1p+rdry11*Y1p+rdry12*Z1p;
                Z1cd = rdry20*X1p+rdry21*Y1p+rdry22*Z1p;
                break;
        case 2: X1cd = rdrz00*X1p+rdrz01*Y1p;
                Y1cd = rdrz10*X1p+rdrz11*Y1p;
                Z1cd = rdrz20*X1p+rdrz21*Y1p;
                break;
        case 3: X1cd = 1; Y1cd = 0; Z1cd = 0; break;
        case 4: X1cd = 0; Y1cd = 1; Z1cd = 0; break;
        case 5: X1cd = 0; Y1cd = 0; Z1cd = 1; break;
      }

      // set jacobian entries (project via K)
      J[(4*i+0)*6+j] = weight*match_param_.f*(X1cd*Z1c-X1c*Z1cd)/(Z1c*Z1c); // left u'
      J[(4*i+1)*6+j] = weight*match_param_.f*(Y1cd*Z1c-Y1c*Z1cd)/(Z1c*Z1c); // left v'
      J[(4*i+2)*6+j] = weight*match_param_.f*(X1cd*Z1c-X2c*Z1cd)/(Z1c*Z1c); // right u'
      J[(4*i+3)*6+j] = weight*match_param_.f*(Y1cd*Z1c-Y1c*Z1cd)/(Z1c*Z1c); // right v'
    }

    // set prediction (project via K)
    p_predict[4*i+0] = match_param_.f*X1c/Z1c+match_param_.cu; // left u
    p_predict[4*i+1] = match_param_.f*Y1c/Z1c+match_param_.cv; // left v
    p_predict[4*i+2] = match_param_.f*X2c/Z1c+match_param_.cu; // right u
    p_predict[4*i+3] = match_param_.f*Y1c/Z1c+match_param_.cv; // right v
    
    // set residuals
    p_residual[4*i+0] = weight*(p_observe[4*i+0]-p_predict[4*i+0]);
    p_residual[4*i+1] = weight*(p_observe[4*i+1]-p_predict[4*i+1]);
    p_residual[4*i+2] = weight*(p_observe[4*i+2]-p_predict[4*i+2]);
    p_residual[4*i+3] = weight*(p_observe[4*i+3]-p_predict[4*i+3]);
  }
}


