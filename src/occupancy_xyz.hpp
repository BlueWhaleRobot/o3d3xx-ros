/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
#ifndef OCCUPANCY_XYZ_H_
#define OCCUPANCY_XYZ_H_

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <ros/console.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/PointField.h>
#include <nav_msgs/OccupancyGrid.h>

//#include <cstdint>  //int8_t
#include <math.h>   //fmod
#include<cstring>
#include<fstream>
#include<iostream>

//line function class,uesed to calculate a point position according to the line,
class LineFunc
{
public:
  LineFunc(double p1[],double p2[])
  {
    if(std::fabs(p1[0]-p2[0])<0.000001)
    {
      A_=0;
      B_=1;
      C_=-p1[0];
    }
    else
    {
      A_=1;
      B_=(p2[1]-p1[1])/(p2[0]-p1[0]);
      C_=p1[1]-B_*p1[0];
    }
  }
  //return value>0 meaning point is locating at left or down side of line ,
  // return value==0 meaning point is in the  line
  //return value<0 meaning point is locating at right or top side of line
  double getPointPos(double p[])
  {
    return A_*p[1]-B_*p[0]-C_;
  }
private:
  double A_;
  double B_;
  double C_;
};

class OccupancyXyz
{
public:
  // Publications
  typedef sensor_msgs::PointCloud2 PointCloud;
  OccupancyXyz()
  {
    init_aready_=false;
  }
  void onInit();
  void depthCb(const sensor_msgs::PointCloud2::Ptr& Incloud_msg);
  void convert2(const sensor_msgs::PointCloud2::Ptr& Incloud_msg,PointCloud::Ptr& cloud_msg);

private:
  double resolution_;  // The map grid resolution [m/cell] ,default 0.1
  std::vector<double>  lowerLeftPos_;// the position of the lower left border  [x,y] ,default [0.4,1.5]
  double gridHeight_; // the  height of map grid ,default 2.0,the width of grid map=2*LowerLeftPos_.y
  double usingWidth_;// the Width length of  lower left border that kinect can seen,default 0.3
  double usingHeight_down_;//the start height length of kinect must seen area
  double usingHeight_up_;//the end height length of kinect must seen area
  int8_t* mapGrid_;// the output occupancy map grid
  float* ZGrid_;
  int8_t* ZGrid2_;
  int8_t* unkown_mapGrid_;// the init output occupancy map grid
  int8_t* filter_mapGrid_;// the output occupancy map grid filter
  std::vector<int> seenGrid_indexs_;
  std::vector<int> mustseenGrid_indexs_;

  std::vector<double> R_;
  std::vector<double> T_;
  double kinectHeight_;
  double barHeight_max_;
  double barHeight_min_;
  ros::Publisher pub_occupancy_grid_;
  ros::Publisher pub_barpoint_cloud_;
  ros::Publisher pub_clearpoint_cloud_;
  ros::Publisher pub_point_cloud_;
  int point_mode_; //0 完整点云、1 除去地板后的所有可视点云
  std::string frame_id_;
  nav_msgs::OccupancyGrid  occupancygrid_msg_;
  int height_temp_;
  int width_temp_;
  // unsigned int num_frame_;
  double bar_threshold_;
  double single_score_;
  double delta_theta_; //value,greater than this consider as bar area
  bool init_aready_;
  double intensity_min_; //value,greater than this consider as valid point
  double (* gridCenters_)[2];
  //ros::NodeHandle nh;
};

void OccupancyXyz::onInit()
{
  ros::NodeHandle nh("~"); // private
  // num_frame_=1;
  // Read parameters
  nh.getParam("lowerLeftPos", lowerLeftPos_);
  nh.getParam("R", R_);
  nh.getParam("T", T_);
  nh.param("resolution", resolution_, 0.1);
  nh.param("gridHeight", gridHeight_, 2.0);
  nh.param("usingWidth", usingWidth_, 0.3);
  nh.param("usingHeight_down",usingHeight_down_, 0.15);
  nh.param("usingHeight_up",usingHeight_up_, 0.3);
  nh.param("frame_id",frame_id_,std::string("kinect_link_new"));
  nh.param("kinectHeight", kinectHeight_, 0.3769);
  nh.param("barHeight_max", barHeight_max_, 0.50);
  nh.param("barHeight_min", barHeight_min_, 0.05);
  nh.param("point_mode", point_mode_, 0);
  nh.param("bar_threshold", bar_threshold_, 10.0);
  nh.param("single_score", single_score_, 2.0);
  nh.param("delta_theta", delta_theta_, 0.015);
  nh.param("intensity_min", intensity_min_,200.0);
  //计算网格大小
  int width_temp,height_temp;

  if(fmod(lowerLeftPos_[1],resolution_)>0.001)
  {
    width_temp=lowerLeftPos_[1]/resolution_+1;
  }
  else
  {
    width_temp=lowerLeftPos_[1]/resolution_;
  }
  if(fmod(gridHeight_,resolution_)>0.001)
  {
    height_temp=gridHeight_/resolution_+1;
  }
  else
  {
    height_temp=gridHeight_/resolution_;
  }
  width_temp =std::max(width_temp,1);
  height_temp =std::max(height_temp,1);
  mapGrid_=new  int8_t[2*height_temp*width_temp];
  ZGrid_=new  float[2*height_temp*width_temp];
  ZGrid2_=new  int8_t[2*height_temp*width_temp];
  unkown_mapGrid_=new  int8_t[2*height_temp*width_temp];
  filter_mapGrid_=new  int8_t[2*height_temp*width_temp];

  int x_num,y_num;
  float x,y; //格子中心点坐标
  double p1[2]={lowerLeftPos_[0],usingWidth_},p2[2]={lowerLeftPos_[0]+gridHeight_,lowerLeftPos_[1]};
  double p3[2]={lowerLeftPos_[0],-usingWidth_},p4[2]={lowerLeftPos_[0]+gridHeight_,-lowerLeftPos_[1]};
  //设置视野边界线
  LineFunc leftborder= LineFunc(p1,p2);
  LineFunc rightborder= LineFunc(p3,p4);
  //初始化网格和视野内网格
  double p[2]={0.0,0.0};
  for(int i=0;i<(2*height_temp*width_temp);i++)
  {
    mapGrid_[i]=-1;//init as Unknown area
    ZGrid_[i]=0.0;
    ZGrid2_[i]=-1;
    unkown_mapGrid_[i]=-1;
    filter_mapGrid_[i]=-1;
    y_num=i/(height_temp);
    x_num=i%(height_temp);
    x=(x_num+0.5)*resolution_+lowerLeftPos_[0];
    y=(-width_temp+y_num+0.5)*resolution_;
    p[0]=x;
    p[1]=y;
    if((leftborder.getPointPos(p)*rightborder.getPointPos(p))>0.000001)
    {
      continue;
    }
    else
    {
      //NODELET_INFO("height_temp: %d  width_temp: %d p1:%f %f p2:%f %f p:%f %f i:%d x_num:%d y_num:%d\n",height_temp,width_temp,p1[0],p1[1],p2[0],p2[1],p[0],p[1],i,x_num,y_num);
      seenGrid_indexs_.push_back(i);
      if(x<(usingHeight_up_+lowerLeftPos_[0]) &&  x>=(usingHeight_down_+lowerLeftPos_[0])) mustseenGrid_indexs_.push_back(i);
    }
  }

  int num=0;
  double* pp;
  gridCenters_=(double (*)[2])new double[seenGrid_indexs_.size()*2];
  for(std::vector<int>::iterator it = seenGrid_indexs_.begin() ; it !=seenGrid_indexs_.end() ; it++)
  {
    int i=*it;
    pp=gridCenters_[num];
    y_num=i/(height_temp);
    x_num=i%(height_temp);
    pp[0]=(x_num+0.5)*resolution_+lowerLeftPos_[0];
    pp[1]=(-width_temp+y_num+0.5)*resolution_;
    //NODELET_INFO("num: %d index: %d x_num: %d y_num: %d\n",num,i,x_num,y_num);
    num++;
  }

  occupancygrid_msg_.info.resolution=resolution_;
  occupancygrid_msg_.info.height=2*width_temp;
  occupancygrid_msg_.info.width=height_temp;
  occupancygrid_msg_.info.origin.position.x=lowerLeftPos_[0];
  occupancygrid_msg_.info.origin.position.y=-width_temp*resolution_;
  occupancygrid_msg_.info.origin.position.z=0;


  height_temp_=height_temp;
  width_temp_=width_temp;
  // Make sure we don't enter connectCb() between advertising and assigning to pub_point_cloud_
  pub_point_cloud_ = nh.advertise<PointCloud>("/kinect/points", 1, true);
  pub_occupancy_grid_ = nh.advertise<nav_msgs::OccupancyGrid>("/kinect/occupancygrid", 1,true);
  pub_barpoint_cloud_ = nh.advertise<PointCloud>("/kinect/barpoints", 1,true);
  pub_clearpoint_cloud_ = nh.advertise<PointCloud>("/kinect/clearpoints", 1, true);

  init_aready_=true;
}


void OccupancyXyz::depthCb(const sensor_msgs::PointCloud2::Ptr& Incloud_msg)
{

  if(!init_aready_) return;

  PointCloud::Ptr cloud_msg(new PointCloud);
  cloud_msg->header = Incloud_msg->header;
  cloud_msg->height = Incloud_msg->height;
  cloud_msg->width  = Incloud_msg->width;
  cloud_msg->is_dense = false;
  cloud_msg->is_bigendian = false;
  cloud_msg->header.frame_id=frame_id_;

  sensor_msgs::PointCloud2Modifier pcd_modifier(*cloud_msg);
  pcd_modifier.setPointCloud2FieldsByString(1, "xyz");

  std::vector<int8_t> a(height_temp_*2*width_temp_,-1);
  occupancygrid_msg_.data = a;
  memcpy(mapGrid_,unkown_mapGrid_,sizeof(int8_t)*(occupancygrid_msg_.info.height*occupancygrid_msg_.info.width));
  memcpy(ZGrid2_,unkown_mapGrid_,sizeof(int8_t)*(occupancygrid_msg_.info.height*occupancygrid_msg_.info.width));

  this->convert2(Incloud_msg, cloud_msg);
  // Time
  ros::Time current_time = ros::Time::now();
  // if((num_frame_%2==0))
  // {
  //   num_frame_++;
  //   return;//10hz
  // }
  // num_frame_++;
  occupancygrid_msg_.header = Incloud_msg->header;
  occupancygrid_msg_.header.frame_id=frame_id_;
  occupancygrid_msg_.info.map_load_time=current_time;
  memcpy(&occupancygrid_msg_.data[0],mapGrid_,sizeof(int8_t)*(occupancygrid_msg_.info.height*occupancygrid_msg_.info.width));
  pub_point_cloud_.publish (cloud_msg);
  pub_occupancy_grid_.publish(occupancygrid_msg_);
}

void OccupancyXyz::convert2(const sensor_msgs::PointCloud2::Ptr& Incloud_msg,PointCloud::Ptr& cloud_msg)
{
  // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)

  float bad_point = std::numeric_limits<float>::quiet_NaN();

  sensor_msgs::PointCloud2Iterator<float> const_iter_In_x(*Incloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> const_iter_In_y(*Incloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> const_iter_In_z(*Incloud_msg, "z");
  sensor_msgs::PointCloud2Iterator<float> const_iter_In_I(*Incloud_msg, "intensity");

  sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_msg, "z");

  float x,y,z;
  float resolution_square=resolution_*resolution_;
  int index_temp1,index_temp2;

  int x_num,y_num;
  float float_temp;
  double * PP;
  for (int v = 0; v < (int)cloud_msg->height; ++v)
  {
    for (int u = 0; u < (int)cloud_msg->width; ++u,++const_iter_In_x, ++const_iter_In_y, ++const_iter_In_z, ++const_iter_In_I, ++iter_x, ++iter_y, ++iter_z)
    {
      // Missing points denoted by NaNs
      if (std::isnan((double)(*const_iter_In_x))||std::isnan((double)(*const_iter_In_y))||std::isnan((double)(*const_iter_In_z)))
      {
        *iter_x = *iter_y = *iter_z = bad_point;
        continue;
      }

      // Fill in XYZ
      x = *const_iter_In_x;
      y = *const_iter_In_y;
      z = *const_iter_In_z;

      *iter_x = R_[0]*x+R_[1]*y+R_[2]*z;
      *iter_y = R_[3]*x+R_[4]*y+R_[5]*z;
      *iter_z = R_[6]*x+R_[7]*y+R_[8]*z+kinectHeight_;

      x=*iter_x;
      y=*iter_y;
      z=*iter_z;

      //地板扣除 障碍物识别
      float barHeight_min_temp,barHeight_max_temp;
      barHeight_max_temp = barHeight_max_;
      barHeight_min_temp = barHeight_min_;

      if(x>0.7)
      {
        barHeight_min_temp = barHeight_min_+x*0.07;
      }
      if(x<1.0)
      {
        barHeight_max_temp = barHeight_max_*0.5;
      }

     if(z< barHeight_max_temp)
     {
       if(z< barHeight_min_temp)
       {
         //地板
         if(point_mode_==1)
         {
           //ROS_INFO("get the floor\n");
           *iter_x = *iter_y = *iter_z = bad_point;
         }
         //除去不感兴趣区域
         if(!((x<(gridHeight_+lowerLeftPos_[0]) &&x>lowerLeftPos_[0])&&(y<lowerLeftPos_[1] && y> -lowerLeftPos_[1]))) continue;

         float_temp=(x-lowerLeftPos_[0])/resolution_;
         x_num=static_cast<int>(float_temp);
         float_temp=y/resolution_+width_temp_;
         y_num=static_cast<int>(float_temp);
         index_temp1=x_num+y_num*height_temp_;
         if(mapGrid_[index_temp1]<0) mapGrid_[index_temp1]=0;
         if(x>(1.0+lowerLeftPos_[0]) || ZGrid2_[index_temp1]>120 ) continue;
         ZGrid2_[index_temp1]+=1;
         if(ZGrid2_[index_temp1]== 0)
         {
           ZGrid_[index_temp1]= z;
         }
         else if( z > ZGrid_[index_temp1])
         {
           ZGrid_[index_temp1] = z;
         }
       }
       else
       {
         //障碍物
         //除去不感兴趣区
          if(!((x<(gridHeight_+lowerLeftPos_[0]) &&x>lowerLeftPos_[0])&&(y<lowerLeftPos_[1] && y> -lowerLeftPos_[1]))) continue;
          float_temp=(x-lowerLeftPos_[0])/resolution_;
          x_num=static_cast<int>(float_temp);
          float_temp=y/resolution_+width_temp_;
          y_num=static_cast<int>(float_temp);
          index_temp1=x_num+y_num*height_temp_;
          mapGrid_[index_temp1]+=single_score_;
          if(mapGrid_[index_temp1]>100) mapGrid_[index_temp1]=100;
       }
     }
    }
  }
  //更新必须看见区域
  for(std::vector<int>::iterator it = mustseenGrid_indexs_.begin() ; it !=mustseenGrid_indexs_.end() ; it++)
  {
    index_temp1=*it;
    if(mapGrid_[index_temp1]<0) //mapGrid_[index_temp1]=100;
    {
      filter_mapGrid_[index_temp1]-=2;
      if(filter_mapGrid_[index_temp1]<-10) filter_mapGrid_[index_temp1]=-10;
    }
    else
    {
      filter_mapGrid_[index_temp1]+=1;
      if(filter_mapGrid_[index_temp1]>0) filter_mapGrid_[index_temp1]=0;
    }
    if(filter_mapGrid_[index_temp1]<-9)
    {
      mapGrid_[index_temp1]=100;
    }
  }
  //更新梯度
  int sum_nums=height_temp_*2*width_temp_;
  int temp_index=-1;
  for(std::vector<int>::iterator it = seenGrid_indexs_.begin() ; it !=seenGrid_indexs_.end() ; it++)
  {
    temp_index++;
    float delta_z=0;
    index_temp1=*it;

    if(ZGrid2_[index_temp1]<0 || mapGrid_[index_temp1]>10 ) continue;

    //梯度检测范围
    PP=gridCenters_[temp_index];
    if(PP[0] > 0.7)
    {
      continue;
    }

    //y_num=index_temp1/(height_temp_);
    x_num=index_temp1%height_temp_;
    // ROS_INFO("get the floor %d \n",ZGrid_[index_temp1]);
    //x方向
    index_temp2=index_temp1-1;
    if(x_num>1 && index_temp2>=0 && index_temp2 < sum_nums && ZGrid2_[index_temp2]>=0)
    {
      if(ZGrid_[index_temp1]<0.02||ZGrid_[index_temp2]<0.02) continue;
      delta_z=ZGrid_[index_temp1]-ZGrid_[index_temp2];
      //if(ZGrid_[index_temp1]>0.01) ROS_INFO("floor bar x %f %f \n",ZGrid_[index_temp1],ZGrid_[index_temp2]);
      if(delta_z>delta_theta_||delta_z<-delta_theta_)
      {
        mapGrid_[index_temp1]=100;
        continue;
      }
    }
    //y方向
    index_temp2=index_temp1 - height_temp_;
    if(index_temp2>=0 && index_temp2 < sum_nums && ZGrid2_[index_temp2]>=0)
    {
      if(ZGrid_[index_temp1]<0.02||ZGrid_[index_temp2]<0.02) continue;

      delta_z=ZGrid_[index_temp1]-ZGrid_[index_temp2];
      //if(ZGrid_[index_temp1]>0.01) ROS_INFO("floor bar y %f %f \n",ZGrid_[index_temp1],ZGrid_[index_temp2]);
      if(delta_z>=delta_theta_||delta_z<=-delta_theta_)
      {
        mapGrid_[index_temp1]=100;
        continue;
      }
    }
  }
  //将可见区域分成可移动区和雷区
  std::vector<int> clearArea,barArea;
  index_temp2=0;
  for(std::vector<int>::iterator it = seenGrid_indexs_.begin() ; it !=seenGrid_indexs_.end() ; it++)
  {
    index_temp1=*it;
    if(mapGrid_[index_temp1]<bar_threshold_&&mapGrid_[index_temp1]>=0)//被遮挡区域不发布
    {
      clearArea.push_back(index_temp2);
    }
    else if(mapGrid_[index_temp1]>=bar_threshold_)
    {
      barArea.push_back(index_temp2);
    }
    index_temp2++;
  }

  if(barArea.size()>1||(barArea.size()==1))//&&num_frame_%4==0))
  {
    //发布雷区
    PointCloud::Ptr barcloud_msg(new PointCloud);
    barcloud_msg->header = Incloud_msg->header;
    barcloud_msg->height = 1;
    barcloud_msg->width  = barArea.size();
    barcloud_msg->is_dense = true;
    barcloud_msg->is_bigendian = false;
    barcloud_msg->header.frame_id=frame_id_;
    sensor_msgs::PointCloud2Modifier pcd_modifier1(*barcloud_msg);
    pcd_modifier1.setPointCloud2FieldsByString(1,"xyz");
    sensor_msgs::PointCloud2Iterator<float> bariter_x(*barcloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> bariter_y(*barcloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> bariter_z(*barcloud_msg, "z");
    for(std::vector<int>::iterator it = barArea.begin() ; it !=barArea.end() ; ++bariter_x, ++bariter_y,++bariter_z,it++)
    {
      index_temp1=*it;
      PP=gridCenters_[index_temp1];
      *bariter_x=PP[0];
      *bariter_y=PP[1];
      *bariter_z=0.15;
    }
    pub_barpoint_cloud_.publish(barcloud_msg);
  }
  // else
  // {
  //   //发布空雷区
  //   PointCloud::Ptr barcloud_msg(new PointCloud);
  //   barcloud_msg->header = Incloud_msg->header;
  //   barcloud_msg->height = 1;
  //   barcloud_msg->width  = 1;
  //   barcloud_msg->is_dense = true;
  //   barcloud_msg->is_bigendian = false;
  //   barcloud_msg->header.frame_id=frame_id_;
  //   sensor_msgs::PointCloud2Modifier pcd_modifier1(*barcloud_msg);
  //   pcd_modifier1.setPointCloud2FieldsByString(1,"xyz");
  //   sensor_msgs::PointCloud2Iterator<float> bariter_x(*barcloud_msg, "x");
  //   sensor_msgs::PointCloud2Iterator<float> bariter_y(*barcloud_msg, "y");
  //   sensor_msgs::PointCloud2Iterator<float> bariter_z(*barcloud_msg, "z");
  //   *bariter_x=-1.0;
  //   *bariter_y=-1.0;
  //   *bariter_z=-1.0;
  //   pub_barpoint_cloud_.publish(barcloud_msg);
  // }
  if(clearArea.size()>0)
  {
    //发布可移动区域
    PointCloud::Ptr clearcloud_msg(new PointCloud);
    clearcloud_msg->header = Incloud_msg->header;
    clearcloud_msg->height = 1;
    clearcloud_msg->width  = clearArea.size();
    clearcloud_msg->is_dense = true;
    clearcloud_msg->is_bigendian = false;
    clearcloud_msg->header.frame_id=frame_id_;
    sensor_msgs::PointCloud2Modifier pcd_modifier2(*clearcloud_msg);
    pcd_modifier2.setPointCloud2FieldsByString(1, "xyz");
    sensor_msgs::PointCloud2Iterator<float> cleariter_x(*clearcloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> cleariter_y(*clearcloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> cleariter_z(*clearcloud_msg, "z");
    for(std::vector<int>::iterator it = clearArea.begin() ; it !=clearArea.end() ; ++cleariter_x, ++cleariter_y,it++)
    {
      index_temp1=*it;
      PP=gridCenters_[index_temp1];
      *cleariter_x=PP[0];
      *cleariter_y=PP[1];
      *cleariter_z=mapGrid_[seenGrid_indexs_[index_temp1]];
    }
    pub_clearpoint_cloud_.publish(clearcloud_msg);
  }

}

#endif /* OCCUPANCY_XYZ_H_ */
