#include "trajectory_generator.h"



trajectory_gen::trajectory_gen(){
  int _poly_order_min = 8;
  int _poly_order_max = 10;
  if(_bernstein.setParam(_poly_order_min, _poly_order_max, 3) == -1) 
  {
    ROS_ERROR(" The trajectory order is set beyond the library's scope, please re-set ");
  }
  traj_solver_server = nh.advertiseService("trajectory_solver", &trajectory_gen::traj_solver_call,this);
  traj_result_server = nh.advertiseService("trajectory_result", &trajectory_gen::traj_out_call,this);
}

double* trajectory_gen::beziersolver10(c_float s_scale, c_float upperPosition,c_float lowerPosition,c_float upperVelocity,
c_float lowerVelocity,c_float upperAccelera,c_float lowerAccelera,c_float x_0,c_float x_n,c_float v_0,c_float v_n){
  
  int _traj_order     = 10;
  
  MatrixXd MQM = _bernstein.getMQM()[_traj_order];
  // cout << MQM <<endl;

  c_float P_var[66];// (n+1)*(n+2)/2
  c_int P_raw[66];// n*(n+1)/2
  c_int P_con[12];// n+2
  P_con[0] = 0;
  int ii_dex = 0;
  for (size_t i = 0; i < (_traj_order+1); i++)
  {
    for (size_t j = 0; j <= i; j++)
    {
      P_var[ii_dex] = MQM(j,i);
      P_raw[ii_dex] = j;
      ii_dex = ii_dex + 1;
    }
    P_con[i+1] = ii_dex;
  }
  c_int   P_mnz  = (_traj_order+1)*(_traj_order+2)/2;
  

  c_float   Pq[11];// n+1
  memset(Pq,0,sizeof(Pq));

  c_float A_var[11];// n+1
  c_int A_raw[11];// n+1
  c_int A_con[12];// n+2
  A_con[0] = 0;
  for (size_t i = 0; i < (_traj_order+1); i++)
  {
    A_var[i]   = 1.0*s_scale;
    A_raw[i]   = i;
    A_con[i+1] = i+1;
  }
  c_int   A_mnz  = (_traj_order+1);// n+1 


  c_int   n_x    = (_traj_order+1);
  c_int   m_x    = (_traj_order+1);

  c_float temp = _traj_order; 
  c_float Av_var[20];//2*n
  c_int Av_raw[20];//2*n
  c_int Av_con[12];// n+2
  c_int Av_mnz  = 2*_traj_order;
  c_int nv_x = (_traj_order+1);// n+1
  c_int mv_x = _traj_order;// n
  ii_dex = 0;
  Av_con[0] = 0;
  for (int i = 0; i < n_x; i++)
  {

    if (i==0)
    {
      Av_var[ii_dex] = 1*temp;
      Av_raw[ii_dex]   = i;
      ii_dex ++;
      Av_con[i+1] = Av_con[i] +1;
    }
    else if (i==n_x-1)
    {
      Av_var[ii_dex] = -1*temp;
      Av_raw[ii_dex]   = i - 1;
      ii_dex ++;
      Av_con[i+1] = Av_con[i] +1;
    }
    else
    {
      Av_var[ii_dex] = -1*temp;
      Av_raw[ii_dex]   = i - 1;
      ii_dex ++;
      Av_var[ii_dex] = 1*temp;
      Av_raw[ii_dex]   = i;
      ii_dex ++;
      Av_con[i+1] = Av_con[i] +2; 
    }
  }
  c_float temp2 = _traj_order*(_traj_order-1)/s_scale; 
  // cout<< "XX:"<< temp2 << "XX" <<endl;
  c_float Aa_var[27];// 3*(n-1)
  c_int Aa_raw[27];
  c_int Aa_con[12];//2*n
  c_int Aa_mnz  = 3*(_traj_order -1);
  c_int na_x = _traj_order+1;
  c_int ma_x = _traj_order - 1;
  ii_dex = 0;
  Aa_con[0] = 0;
  for (int i = 0; i < n_x; i++)
  {

    if (i==0)
    {
      Aa_var[ii_dex] = 1*temp2;
      Aa_raw[ii_dex]   = i;
      ii_dex ++;
      Aa_con[i+1] = Aa_con[i] +1;
    }
    else if (i==1)
    {
      Aa_var[ii_dex] = -2*temp2;
      Aa_raw[ii_dex]   = i - 1;
      ii_dex ++;
      Aa_var[ii_dex] = 1*temp2;
      Aa_raw[ii_dex]   = i;
      ii_dex ++;
      Aa_con[i+1] = Aa_con[i] +2; 
    }
    else if (i==n_x-2)
    {
      Aa_var[ii_dex] = 1*temp2;
      Aa_raw[ii_dex]   = i - 2;
      ii_dex ++;
      Aa_var[ii_dex] = -2*temp2;
      Aa_raw[ii_dex]   = i - 1;
      ii_dex ++;
      Aa_con[i+1] = Aa_con[i] +2; 
    }
    else if (i==n_x-1)
    {
      Aa_var[ii_dex] = -1*temp2;
      Aa_raw[ii_dex]   = i - 2;
      ii_dex ++;
      Aa_con[i+1] = Aa_con[i] +1;
    }
    else
    {
      Aa_var[ii_dex] = 1*temp2;
      Aa_raw[ii_dex]   = i - 2;
      ii_dex ++;
      Aa_var[ii_dex] = -2*temp2;
      Aa_raw[ii_dex]   = i - 1;
      ii_dex ++;
      Aa_var[ii_dex] = 1*temp2;
      Aa_raw[ii_dex]   = i;
      ii_dex ++;
      Aa_con[i+1] = Aa_con[i] +3; 
    }
  }
  
 
  // position + velocity
  c_int C_nzmax;
  c_int C_n;
  c_int C_m;
  c_float var[31];// _traj_order+1 + 2*_traj_order
  c_int   p[12];
  c_int ii[31];
   csc_combine( A_mnz, n_x,  m_x, A_con, A_raw,A_var,
    Av_mnz, nv_x,  mv_x, Av_con, Av_raw,Av_var,
    C_nzmax,  C_n, C_m, p, ii, var);
  // csc* CC = csc_matrix(C_m, C_n, C_nzmax, var, ii, p);

  // position + velocity + acceleration
  c_int CC_nzmax;
  c_int CC_n;
  c_int CC_m;
  c_float CCvar[58];// _traj_order+1 + 2*_traj_order + 3*(_traj_order-1) 
  c_int   CCp[12];
  c_int CCii[58];
  csc_combine( C_nzmax,  C_n, C_m,  p, ii, var,
      Aa_mnz, na_x,  ma_x, Aa_con, Aa_raw,Aa_var,
      CC_nzmax,  CC_n, CC_m,  CCp, CCii, CCvar);

  csc* CC = csc_matrix(CC_m, CC_n, CC_nzmax, CCvar, CCii, CCp);

  
    // position + velocity + acceleration
    c_float l_a[30];//  _traj_order+1 + _traj_order + _traj_order -1
    c_float u_a[30];
    for (size_t i = 0; i < (_traj_order +1); i++)
    {
      l_a[i] = lowerPosition;
      u_a[i] = upperPosition;
    }
    for (size_t i = (_traj_order +1); i < (_traj_order+1 + _traj_order ); i++)
    {
      l_a[i] = lowerVelocity;
      u_a[i] = upperVelocity;
    }
    for (size_t i = (_traj_order+1 + _traj_order ); i < (3*_traj_order); i++)
    {
      l_a[i] = lowerAccelera;
      u_a[i] = upperAccelera;
    }
    
    l_a[0] = x_0;
    u_a[0] = x_0;
    l_a[_traj_order] = x_n;
    u_a[_traj_order] = x_n;
    l_a[_traj_order +1] = v_0;
    u_a[_traj_order +1] = v_0;
    l_a[2*_traj_order] = v_n;
    u_a[2*_traj_order] = v_n;

    
    
    // Exitflag
    c_int exitflag = 0;

    // Workspace structures
    OSQPWorkspace *work;
    OSQPSettings  *settings = (OSQPSettings *)c_malloc(sizeof(OSQPSettings));
    OSQPData      *data     = (OSQPData *)c_malloc(sizeof(OSQPData));

    // Populate data
    if (data) {
      data->n = n_x;
      data->m = CC->m;
      data->P = csc_matrix(data->n, data->n, P_mnz, P_var, P_raw, P_con); 
      data->q = Pq;
      data->A = CC;
      data->l = l_a;
      data->u = u_a;
    }
   

    // Define solver settings as default
    if (settings) osqp_set_default_settings(settings);

    // Setup workspace
    exitflag = osqp_setup(&work, data, settings);

    // Solve Problem
    osqp_solve(work);
    cout << "Result = " << work->solution->x[0] <<endl;
    cout << "Result = " << work->solution->x[1] <<endl;
    cout << "Result = " << work->solution->x[2] <<endl;
    cout << "Result = " << work->solution->x[3] <<endl;
    cout << "Result = " << work->solution->x[4] <<endl;
    cout << "Result = " << work->solution->x[5] <<endl;
    cout << "Result = " << work->solution->x[6] <<endl;
    cout << "Result = " << work->solution->x[7] <<endl;
    cout << "Result = " << work->solution->x[8] <<endl;
    cout << "Result = " << work->solution->x[9] <<endl;
    cout << "Result = " << work->solution->x[10] <<endl;
    

    double* out = new double[11];
    for (size_t i = 0; i < (_traj_order+1); i++)
    {
      out[i] = work->solution->x[i]; 
    }
    
    

    // Clean workspace
    osqp_cleanup(work);
    if (data) {
      if (data->A) c_free(data->A);
      if (data->P) c_free(data->P);
      c_free(data);
    }
    if (settings)  c_free(settings);
    //ROS_INFO("exitflag=%d",exitflag);

    return out;
}

double* trajectory_gen::beziersolver8(c_float s_scale, c_float upperPosition,c_float lowerPosition,c_float upperVelocity,
c_float lowerVelocity,c_float upperAccelera,c_float lowerAccelera,c_float x_0,c_float x_n,c_float v_0,c_float v_n){
  int _traj_order     = 8;
  MatrixXd MQM = _bernstein.getMQM()[_traj_order];
  // cout << MQM <<endl;

  c_float P_var[45];// (n+1)*(n+2)/2
  c_int P_raw[45];// n*(n+1)/2
  c_int P_con[10];// n+2
  P_con[0] = 0;
  int ii_dex = 0;
  for (size_t i = 0; i < (_traj_order+1); i++)
  {
    for (size_t j = 0; j <= i; j++)
    {
      P_var[ii_dex] = MQM(j,i);
      P_raw[ii_dex] = j;
      ii_dex = ii_dex + 1;
    }
    P_con[i+1] = ii_dex;
  }
  c_int   P_mnz  = (_traj_order+1)*(_traj_order+2)/2;
  

  c_float   Pq[9];// n+1
  memset(Pq,0,sizeof(Pq));

  c_float A_var[9];// n+1
  c_int A_raw[9];// n+1
  c_int A_con[10];// n+2
  A_con[0] = 0;
  for (size_t i = 0; i < (_traj_order+1); i++)
  {
    A_var[i]   = 1.0*s_scale;
    A_raw[i]   = i;
    A_con[i+1] = i+1;
  }
  c_int   A_mnz  = (_traj_order+1);// n+1 


  c_int   n_x    = (_traj_order+1);
  c_int   m_x    = (_traj_order+1);

  c_float temp = _traj_order; 
  c_float Av_var[16];//2*n
  c_int Av_raw[16];//2*n
  c_int Av_con[10];// n+2
  c_int Av_mnz  = 2*_traj_order;
  c_int nv_x = (_traj_order+1);// n+1
  c_int mv_x = _traj_order;// n
  ii_dex = 0;
  Av_con[0] = 0;
  for (int i = 0; i < n_x; i++)
  {

    if (i==0)
    {
      Av_var[ii_dex] = 1*temp;
      Av_raw[ii_dex]   = i;
      ii_dex ++;
      Av_con[i+1] = Av_con[i] +1;
    }
    else if (i==n_x-1)
    {
      Av_var[ii_dex] = -1*temp;
      Av_raw[ii_dex]   = i - 1;
      ii_dex ++;
      Av_con[i+1] = Av_con[i] +1;
    }
    else
    {
      Av_var[ii_dex] = -1*temp;
      Av_raw[ii_dex]   = i - 1;
      ii_dex ++;
      Av_var[ii_dex] = 1*temp;
      Av_raw[ii_dex]   = i;
      ii_dex ++;
      Av_con[i+1] = Av_con[i] +2; 
    }
  }
  c_float temp2 = _traj_order*(_traj_order-1)/s_scale; 
  cout<< "XX:"<< temp2 << "XX" <<endl;
  c_float Aa_var[21];// 3*(n-1)
  c_int Aa_raw[21];
  c_int Aa_con[10];//n+2
  c_int Aa_mnz  = 3*(_traj_order -1);
  c_int na_x = _traj_order+1;
  c_int ma_x = _traj_order - 1;
  ii_dex = 0;
  Aa_con[0] = 0;
  for (int i = 0; i < n_x; i++)
  {

    if (i==0)
    {
      Aa_var[ii_dex] = 1*temp2;
      Aa_raw[ii_dex]   = i;
      ii_dex ++;
      Aa_con[i+1] = Aa_con[i] +1;
    }
    else if (i==1)
    {
      Aa_var[ii_dex] = -2*temp2;
      Aa_raw[ii_dex]   = i - 1;
      ii_dex ++;
      Aa_var[ii_dex] = 1*temp2;
      Aa_raw[ii_dex]   = i;
      ii_dex ++;
      Aa_con[i+1] = Aa_con[i] +2; 
    }
    else if (i==n_x-2)
    {
      Aa_var[ii_dex] = 1*temp2;
      Aa_raw[ii_dex]   = i - 2;
      ii_dex ++;
      Aa_var[ii_dex] = -2*temp2;
      Aa_raw[ii_dex]   = i - 1;
      ii_dex ++;
      Aa_con[i+1] = Aa_con[i] +2; 
    }
    else if (i==n_x-1)
    {
      Aa_var[ii_dex] = -1*temp2;
      Aa_raw[ii_dex]   = i - 2;
      ii_dex ++;
      Aa_con[i+1] = Aa_con[i] +1;
    }
    else
    {
      Aa_var[ii_dex] = 1*temp2;
      Aa_raw[ii_dex]   = i - 2;
      ii_dex ++;
      Aa_var[ii_dex] = -2*temp2;
      Aa_raw[ii_dex]   = i - 1;
      ii_dex ++;
      Aa_var[ii_dex] = 1*temp2;
      Aa_raw[ii_dex]   = i;
      ii_dex ++;
      Aa_con[i+1] = Aa_con[i] +3; 
    }
  }
  
 
  // position + velocity
  c_int C_nzmax;
  c_int C_n;
  c_int C_m;
  c_float var[25];// _traj_order+1 + 2*_traj_order
  c_int   p[10];// n+2
  c_int ii[25];
   csc_combine( A_mnz, n_x,  m_x, A_con, A_raw,A_var,
    Av_mnz, nv_x,  mv_x, Av_con, Av_raw,Av_var,
    C_nzmax,  C_n, C_m, p, ii, var);
  // csc* CC = csc_matrix(C_m, C_n, C_nzmax, var, ii, p);

  // position + velocity + acceleration
    c_int CC_nzmax;
    c_int CC_n;
    c_int CC_m;
    c_float CCvar[46];// _traj_order+1 + 2*_traj_order + 3*(_traj_order-1) 
    c_int   CCp[10];
    c_int CCii[46];
    csc_combine( C_nzmax,  C_n, C_m,  p, ii, var,
        Aa_mnz, na_x,  ma_x, Aa_con, Aa_raw,Aa_var,
        CC_nzmax,  CC_n, CC_m,  CCp, CCii, CCvar);

    csc* CC = csc_matrix(CC_m, CC_n, CC_nzmax, CCvar, CCii, CCp);
    c_float l_a[24];//  _traj_order+1 + _traj_order + _traj_order -1
    c_float u_a[24];
    for (size_t i = 0; i < (_traj_order +1); i++)
    {
      l_a[i] = lowerPosition;
      u_a[i] = upperPosition;
    }
    for (size_t i = (_traj_order +1); i < (_traj_order+1 + _traj_order ); i++)
    {
      l_a[i] = lowerVelocity;
      u_a[i] = upperVelocity;
    }
    for (size_t i = (_traj_order+1 + _traj_order ); i < (3*_traj_order); i++)
    {
      l_a[i] = lowerAccelera;
      u_a[i] = upperAccelera;
    }
    
    l_a[0] = x_0;
    u_a[0] = x_0;
    l_a[_traj_order] = x_n;
    u_a[_traj_order] = x_n;
    l_a[_traj_order +1] = v_0;
    u_a[_traj_order +1] = v_0;
    l_a[2*_traj_order] = v_n;
    u_a[2*_traj_order] = v_n;
    // vector_show(u_a,24);
    // csc_show(CC);
    
    
  
    
    // Exitflag
    c_int exitflag = 0;

    // Workspace structures
    OSQPWorkspace *work;
    OSQPSettings  *settings = (OSQPSettings *)c_malloc(sizeof(OSQPSettings));
    OSQPData      *data     = (OSQPData *)c_malloc(sizeof(OSQPData));

    // Populate data
    if (data) {
      data->n = n_x;
      data->m = CC->m;
      data->P = csc_matrix(data->n, data->n, P_mnz, P_var, P_raw, P_con);
      data->q = Pq;
      data->A = CC;
      data->l = l_a;
      data->u = u_a;
    }

    // Define solver settings as default
    if (settings) osqp_set_default_settings(settings);

    // Setup workspace
    exitflag = osqp_setup(&work, data, settings);

    // Solve Problem
    osqp_solve(work);
    double* out = new double[9];
    for (size_t i = 0; i < (_traj_order+1); i++)
    {
      out[i] = work->solution->x[i]; 
    }

    // Clean workspace
    osqp_cleanup(work);
    if (data) {
      if (data->A) c_free(data->A);
      if (data->P) c_free(data->P);
      c_free(data);
    }
    if (settings)  c_free(settings);

    return out;
}

float* trajectory_gen::bezierout(bezier _bezier,ros::Time timeNow){
  VectorXd CList  = _bernstein.getC()[_bezier.order];
  VectorXd CvList = _bernstein.getC_v()[_bezier.order];
  VectorXd CaList = _bernstein.getC_a()[_bezier.order];
  static float out[3];
  float t =  (timeNow.toSec() - _bezier.times.toSec())/_bezier.scale;
  // printf("%f\t",t);
  if (t>1)
  {
    ROS_WARN("Beyond time scope！\n");
  }
  
  memset(out,0,sizeof(out));
  if (timeNow.toSec()-_bezier.times.toSec()>_bezier.scale)
  {
    out[0] =  _bezier.scale *_bezier.coef[_bezier.order] ;
    out[1] =  _bezier.order * (_bezier.coef[_bezier.order] - _bezier.coef[_bezier.order-1]) ;
    out[2] =  1.0 / _bezier.scale * _bezier.order * (_bezier.order - 1) 
                * (_bezier.coef[_bezier.order]  - 2*_bezier.coef[_bezier.order-1]  + _bezier.coef[_bezier.order-2] );
  }
  else
  {
      for (size_t i = 0; i < (_bezier.order+1); i++)
    {
      out[0]  += _bezier.scale *_bezier.coef[i] * CList(i) * pow(t, i) * pow((1 - t), (_bezier.order - i) );
      if (i<_bezier.order)
      {
        out[1] += CvList(i) * _bezier.order * (_bezier.coef[i+1] - _bezier.coef[i]) * pow(t, i) * pow((1 - t), (_bezier.order - 1 - i) );
      }
      if (i<_bezier.order-1)
      {
        out[2] += 1.0 / _bezier.scale * CaList(i) * _bezier.order * (_bezier.order - 1) 
                * (_bezier.coef[i+2]  - 2*_bezier.coef[i+1]  + _bezier.coef[i] ) * pow(t, i) * pow((1 - t), (_bezier.order - 2 - i) );
      }
    }
  }
  return out;
}
bool trajectory_gen::traj_solver_call(trajectory_generation::traj_solver_msgRequest& request,trajectory_generation::traj_solver_msgResponse& response){
  c_float s_scale       = request.s_scale;
  c_float upperPosition = request.x_upperPosition;
  c_float lowerPosition = request.x_lowerPosition;
  c_float upperVelocity = request.x_upperVelocity;
  c_float lowerVelocity = request.x_lowerVelocity;
  c_float upperAccelera = request.x_upperAccelera;
  c_float lowerAccelera = request.x_lowerAccelera;
  c_float x_0 = request.x_0;
  c_float x_n = request.x_n;
  c_float v_0 = request.x_v0;
  c_float v_n = request.x_vn;

  printf("trajectory_generation:   x0=%f \t y0=%f \t z0=%f \n",request.x_0,request.y_0,request.z_0);

  _bezier_x.coef  = beziersolver8(s_scale,upperPosition,lowerPosition,upperVelocity,lowerVelocity,
                upperAccelera,lowerAccelera,x_0,x_n,v_0,v_n);
  _bezier_x.order  = 8;
  _bezier_x.scale  = s_scale;
  _bezier_x.times = ros::Time::now();

  upperPosition = request.y_upperPosition;
  lowerPosition = request.y_lowerPosition;
  upperVelocity = request.y_upperVelocity;
  lowerVelocity = request.y_lowerVelocity;
  upperAccelera = request.y_upperAccelera;
  lowerAccelera = request.y_lowerAccelera;
  x_0 = request.y_0;
  x_n = request.y_n;
  v_0 = request.y_v0;
  v_n = request.y_vn;

  _bezier_y.coef  = beziersolver8(s_scale,upperPosition,lowerPosition,upperVelocity,lowerVelocity,
                upperAccelera,lowerAccelera,x_0,x_n,v_0,v_n);
  _bezier_y.order  = 8;
  _bezier_y.scale  = s_scale;
  _bezier_y.times = ros::Time::now();

  upperPosition = request.z_upperPosition;
  lowerPosition = request.z_lowerPosition;
  upperVelocity = request.z_upperVelocity;
  lowerVelocity = request.z_lowerVelocity;
  upperAccelera = request.z_upperAccelera;
  lowerAccelera = request.z_lowerAccelera;
  x_0 = request.z_0;
  x_n = request.z_n;
  v_0 = request.z_v0;
  v_n = request.z_vn;

  _bezier_z.coef  = beziersolver8(s_scale,upperPosition,lowerPosition,upperVelocity,lowerVelocity,
                upperAccelera,lowerAccelera,x_0,x_n,v_0,v_n);
  _bezier_z.order  = 8;
  _bezier_z.scale  = s_scale;
  _bezier_z.times = ros::Time::now();
  return 1;
}

bool trajectory_gen::traj_out_call(trajectory_generation::traj_out_msgRequest& request,trajectory_generation::traj_out_msgResponse& response){
  float* outShow;
  outShow       = bezierout(_bezier_x, request.times);
  response.x    = outShow[0];
  response.dx   = outShow[1];
  response.ddx  = outShow[2]; 
  outShow       = bezierout(_bezier_y, request.times);
  response.y    = outShow[0];
  response.dy   = outShow[1];
  response.ddy  = outShow[2]; 
  outShow       = bezierout(_bezier_z, request.times);
  response.z    = outShow[0];
  response.dz   = outShow[1];
  response.ddz  = outShow[2]; 
  ROS_INFO("%f\t%f\t%f\n",response.x,response.y,response.z);
  return 1;
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "traj_gen");
  ROS_INFO("Read to solve trajectory！");
  trajectory_gen _trajectory_gen_;
  ros::spin();
  return 0;
}