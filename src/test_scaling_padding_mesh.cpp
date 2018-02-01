#include <ros/ros.h>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/mesh_operations.h>
#include <fstream>

/**
 * @function main
 */
int main( int argc, char* argv[] ) {

  ros::init( argc, argv, "test_scaling_padding" );
  ros::NodeHandle nh;

  std::string mesh_filename( "package://denso_vs068_with_gripper_gazebo/models/lab/traclabs_setup_table.dae" );
  double scale = 1.0;
  double padding = 0.05;
  
  
  nh.getParam( ros::this_node::getName() + "/mesh_path", mesh_filename );
  ROS_INFO("\t * Mesh filename: %s ", mesh_filename.c_str() );

  nh.getParam( ros::this_node::getName() + "/scale", scale );
  ROS_INFO("\t * Mesh scale: %f ", scale );

  nh.getParam( ros::this_node::getName() + "/padding", padding );
  ROS_INFO("\t * Mesh padding: %f ", padding );
  
  
  // 1. Get shape from file
  shapes::Shape* mesh_shape = NULL;
  shapes::Shape* mesh_shape_copy = NULL;
  mesh_shape = shapes::createMeshFromResource( mesh_filename );
  if( !mesh_shape ) {
    ROS_INFO("Failed parsing mesh\n");
    return 1;
  }
  mesh_shape_copy = shapes::createMeshFromResource( mesh_filename );
  
  std::string original_mesh("/home/ana/original_mesh.stl");;
  std::string padded_mesh("/home/ana/padded_mesh.stl");
  std::string new_padded_mesh("/home/ana/new_padded_mesh.stl");
  std::vector<char> original_buffer, padded_buffer, new_padded_buffer;

  
  // Store original  
  writeSTLBinary( dynamic_cast<shapes::Mesh*>(mesh_shape), original_buffer );
  std::ofstream outfile_original( original_mesh.c_str(), std::ofstream::binary );
  outfile_original.write( &original_buffer[0], original_buffer.size() );
  outfile_original.close();

  // Store padded (default way)  
  mesh_shape->padd( padding );

  writeSTLBinary( dynamic_cast<shapes::Mesh*>(mesh_shape), padded_buffer );
  std::ofstream outfile_padded( padded_mesh.c_str(), std::ofstream::binary );
  outfile_padded.write( &padded_buffer[0], padded_buffer.size() );
  outfile_padded.close();
  
  // Store padded (new fatten way)
  (dynamic_cast<shapes::Mesh*>(mesh_shape_copy))->padd_fatten( padding );

  writeSTLBinary( dynamic_cast<shapes::Mesh*>(mesh_shape_copy), new_padded_buffer );
  std::ofstream outfile_new_padded( new_padded_mesh.c_str(), std::ofstream::binary );
  outfile_new_padded.write( &new_padded_buffer[0], new_padded_buffer.size() );
  outfile_new_padded.close();

  
  return 0;
  
}
