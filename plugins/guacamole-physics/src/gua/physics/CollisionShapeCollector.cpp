#include <gua/physics/CollisionShapeCollector.hpp>


namespace gua{

CollisionShapeCollector::CollisionShapeCollector():
	collected_collision_shapes_(std::list<std::tuple<std::shared_ptr<physics::CollisionShapeNode>,math::mat4,float>>())
	{}

CollisionShapeCollector::~CollisionShapeCollector(){}


void
CollisionShapeCollector::check(PhysicalNode* pn){

	collected_collision_shapes_.clear();
	// visit the rigid body
  	pn->accept(*this);

}

void
CollisionShapeCollector::visit(PhysicalNode* node){
	if(node->is_collidable()){
		auto cs = node->get_collision_shape();
		if(cs){	
			
			auto transform = node->get_geometry()->get_world_transform();

			collected_collision_shapes_.push_back(std::make_tuple(cs,transform,node->get_mass()));
		}
		else{
			std::cout<<"no cs :(((("<<std::endl;
		}
		for(auto const& child : node->get_children()){
			child->accept(*this);
		}
	}

}

math::vec3
CollisionShapeCollector::get_center_of_mass()const{

	float new_mass = 0.0;
	math::vec3 com = math::vec3(0.0,0.0,0.0); //center of mass

	for(auto cs : collected_collision_shapes_){
		auto mat = std::get<1>(cs);
		auto mass = std::get<2>(cs);
		new_mass += mass;
		com += math::vec3(mat[12],mat[13],mat[14]) * mass;
	}


	return com / new_mass;
}

void
CollisionShapeCollector::add_shapes_to_rb(physics::RigidBodyNode* rb){
	for(auto cs : collected_collision_shapes_){

		auto cs_world = std::get<1>(cs);

		//getScale solution from avango-gua
		math::vec3 x_vec(cs_world[0], cs_world[1], cs_world[2]);
	    math::vec3 y_vec(cs_world[4], cs_world[5], cs_world[6]);
	    math::vec3 z_vec(cs_world[8], cs_world[9], cs_world[10]);
	    auto cs_scale = math::vec3(scm::math::length(x_vec), scm::math::length(y_vec), scm::math::length(z_vec));

		//similar to transformation of own cs:
		std::get<0>(cs)->set_transform(scm::math::inverse(rigid_body_->get_transform()) * cs_world * scm::math::inverse(scm::math::make_scale(cs_scale)));//all other collision shapes in lower graph must be in rb coord.syst.

		rigid_body_->add_child(std::get<0>(cs));
	}
}


}
