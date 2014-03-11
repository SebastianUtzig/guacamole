// class header
#include <gua/physics/PhysicalNode.hpp>


namespace gua{


PhysicalNode::PhysicalNode(/*std::string const& name,
				 physics::Physics* physics,
                 GeometryNode::Configuration const& configuration,
                 math::mat4 const& transform,*/
                 std::shared_ptr<GeometryNode> const& geom,
                 physics::Physics* physics,
				 std::shared_ptr<physics::CollisionShapeNode> const& cs,
                 float mass,
                 float friction,
                 float restitution)
		: 	//GeometryNode(name,configuration,transform),
			TransformNode(geom->get_name()+"_physical"),
			rigid_body_(nullptr),
			mass_(mass),
			friction_(friction),
			restitution_(restitution),
			collision_shape_(cs),
			physics_(physics),
			geometry_(geom),
			scale_(math::vec3(1.0,1.0,1.0)),
			set_scale_(false)
		{
			add_child(geometry_);
		}



bool
PhysicalNode::make_collidable(bool b_make_collidable,bool warn_parent){

	if(b_make_collidable){

		if(rigid_body_==nullptr){


			auto geom_world_ = geometry_->get_world_transform();
			//std::cout<<"geom world of: "<<get_geometry()->get_path()<<geom_world_<<std::endl;
			//std::cout<<"own world of: "<<get_path()<<get_transform()<<std::endl;



			if(/*mass_!=0.0 &&*/ set_scale_==false){
				//getScale solution from avango-gua
				math::vec3 x_vec(geom_world_[0], geom_world_[1], geom_world_[2]);
			    math::vec3 y_vec(geom_world_[4], geom_world_[5], geom_world_[6]);
			    math::vec3 z_vec(geom_world_[8], geom_world_[9], geom_world_[10]);
			    scale_ = math::vec3(scm::math::length(x_vec), scm::math::length(y_vec), scm::math::length(z_vec));
			    set_scale_ = true;
			}


			//create collision shape
			if (collision_shape_ == nullptr){
				calculate_collision_shape();
			}

			
			//collect all collision shapes of subgraph
			auto collision_shapes = std::list<std::tuple<std::shared_ptr<physics::CollisionShapeNode>,math::mat4,float>>();


			for(auto const& child : get_children()){
				collect_collision_shapes(&*child,collision_shapes);
			}

			//create physics subgraph: (root - rb - cs)
			///root:
			auto phys_root = new TransformNode("phys_root");

			///rigidbody and collisionshape of first geometry:

			//rigidbody just consist of center of mass translation - offset in collisionshape and geometry
			if(mass_ != 0){
				//calculate new center of mass
				float new_mass = mass_;
				math::vec3 com = math::vec3(0.0,0.0,0.0); //center of mass
				com += math::vec3(geom_world_[12],geom_world_[13],geom_world_[14]) * mass_;
				for(auto cs : collision_shapes){
					auto mat = std::get<1>(cs);
					auto mass = std::get<2>(cs);
					new_mass += mass;
					com += math::vec3(mat[12],mat[13],mat[14]) * mass;
				}


				com = com / new_mass;

				rigid_body_= std::shared_ptr<physics::RigidBodyNode>(new physics::RigidBodyNode(get_name()+"_rb_",mass_,friction_,restitution_,scm::math::make_translation(com)));
				

				set_transform(scm::math::make_translation(com));


				collision_shape_->set_transform(scm::math::inverse(rigid_body_->get_transform()) * geom_world_* scm::math::inverse(scm::math::make_scale(scale_)));

				geometry_->set_transform(scm::math::inverse(scm::math::make_translation(com)) * geom_world_);//* scm::math::inverse(scm::math::make_scale(scale_)));// * scm::math::make_scale(scale_));
				


			}
			else{//static object can hold all transformations of upper geometry

				rigid_body_= std::shared_ptr<physics::RigidBodyNode>(new physics::RigidBodyNode(get_name()+"_rb_",mass_,friction_,restitution_,geom_world_ * scm::math::inverse(scm::math::make_scale(scale_))));


				set_transform(geom_world_);


				collision_shape_->set_transform(math::mat4::identity());

				geometry_->set_transform(math::mat4::identity());
			}



			rigid_body_->add_child(collision_shape_);

			//add to physics root:
			phys_root->add_child(rigid_body_);

//			if(warn_parent)warn_parent_physics(get_parent());
 

			//std::cout<<"collision_shapes length "<<collision_shapes.size()<<std::endl;
			for(auto cs : collision_shapes){

				//similar to transformation of own cs:
				std::get<0>(cs)->set_transform(scm::math::inverse(rigid_body_->get_transform()) * std::get<1>(cs) * scm::math::inverse(scm::math::make_scale(scale_)));//all other collision shapes in lower graph must be in rb coord.syst.

				rigid_body_->add_child(std::get<0>(cs));
			}



			physics_->add_rigid_body(std::make_pair(rigid_body_,this));
			
			if(warn_parent)warn_parent_physics(get_parent());
		}
		else{
			return update_physics_structure();
		}
	}
	else if(rigid_body_){
		physics_->remove_rigid_body(rigid_body_);

		//set all transformations into geometry again
		auto parent_trans = get_parent()->get_world_transform();
		geometry_->set_transform(scm::math::inverse(parent_trans) * get_transform()  * geometry_->get_transform());
		set_transform(math::mat4::identity());

		//try to rescue cs (???)
		//rigid_body_->clear_children();
		rigid_body_.reset();

		rigid_body_ = nullptr;
		if(warn_parent)warn_parent_physics(get_parent());
	}

	return true;

}


bool
PhysicalNode::is_collidable()const{
	if (rigid_body_){
		return true;
	}
	return false;
}


void
PhysicalNode::set_world_transform(math::mat4 const& transform){
	if(mass_ == 0){
		set_transform(transform * scm::math::make_scale(scale_));
	}
	else{
		set_transform(transform);
	} 
}

math::mat4
PhysicalNode::get_world_transform()const{
	if(!is_collidable()){
		if (parent_){
	        return parent_->get_world_transform() * get_transform();
	    }
	    return get_transform();
	}
	else{
    	return get_transform();
	}
}

void
PhysicalNode::calculate_collision_shape(){
	
	// std::cout<<"Attention: Trying to build CollisionShape automatically!!!"<<std::endl;

	std::string cs_name = geometry_->get_name()
					+"_automatic_collision_shape_"
					+gua::string_utils::to_string(scale_.x)+"_"
					+gua::string_utils::to_string(scale_.y)+"_"
					+gua::string_utils::to_string(scale_.z);

	auto existing_cs = gua::physics::CollisionShapeDatabase::instance()->lookup(cs_name);
	if(existing_cs == nullptr){

		std::vector<std::string> geometry_list = std::vector<std::string>();
		//geometry_list.push_back(geometry_->get_name());
		geometry_list.push_back(geometry_->data.get_geometry());
		auto cs = gua::physics::TriangleMeshShape::FromGeometry(geometry_list, true, true);
		cs->set_scaling(scale_);
		gua::physics::CollisionShapeDatabase::add_shape(cs_name, cs);

	}	
	std::shared_ptr<gua::physics::CollisionShapeNode> csn (new gua::physics::CollisionShapeNode(cs_name));
	csn->data.set_shape(cs_name);
	collision_shape_ = csn;
}

std::shared_ptr<physics::CollisionShapeNode>
PhysicalNode::get_collision_shape(){
	if (collision_shape_){return collision_shape_;}
	else {
		auto geom_world(geometry_->get_world_transform());
		math::vec3 x_vec(geom_world[0], geom_world[1], geom_world[2]);
	    math::vec3 y_vec(geom_world[4], geom_world[5], geom_world[6]);
	    math::vec3 z_vec(geom_world[8], geom_world[9], geom_world[10]);
	    scale_ = math::vec3(scm::math::length(x_vec), scm::math::length(y_vec), scm::math::length(z_vec));
	    set_scale_ = true;
		calculate_collision_shape();
		return collision_shape_;
	}
}

std::shared_ptr<GeometryNode>
PhysicalNode::get_geometry()const{
	return geometry_;
}

float
PhysicalNode::get_mass()const{
	return mass_;
}



void
PhysicalNode::update_cache(){
	if(!is_collidable()){
		if (self_dirty_) {
	        if (is_root()) {
	            world_transform_ = get_transform();
	        } else {
	            world_transform_ = get_parent()->get_cached_world_transform() * get_transform();
	        }
	        self_dirty_ = false;
	    }
	    if (child_dirty_) {
	        for (auto child: get_children()) {
	            child->update_cache();
	        }
	        update_bounding_box();
	        child_dirty_ = false;
	    }
	}
	else{
		if (self_dirty_) {
	       	world_transform_ = get_transform();
	        self_dirty_ = false;
	    }

	    if (child_dirty_) {
	        for (auto child: get_children()) {
	            child->update_cache();
	        }
	        update_bounding_box();
	        child_dirty_ = false;
	    }
	}
}



void
PhysicalNode::collect_collision_shapes(Node* node,std::list<std::tuple<std::shared_ptr<physics::CollisionShapeNode>,math::mat4,float>>& collision_shapes)const{
	//auto phys_node = std::dynamic_pointer_cast<PhysicalNode>(node);
	auto phys_node = dynamic_cast<PhysicalNode*>(node);
	if(phys_node){
		if(!phys_node -> is_collidable()){
			auto cs = phys_node->get_collision_shape();
			if(cs){	
				
				//collision_shapes.push_back(std::make_pair(cs,cs->get_world_transform()));
				//getScale solution from avango-gua
				auto transform = phys_node->get_geometry()->get_world_transform();
				/*math::vec3 x_vec(transform[0], transform[1], transform[2]);
			    math::vec3 y_vec(transform[4], transform[5], transform[6]);
			    math::vec3 z_vec(transform[8], transform[9], transform[10]);
			    auto scale = math::vec3(scm::math::length(x_vec), scm::math::length(y_vec), scm::math::length(z_vec));
			    auto without_scale = scm::math::inverse(scm::math::make_scale(scale) * transform);*/

				collision_shapes.push_back(std::make_tuple(cs,transform,phys_node->get_mass()));
			}
			else{
				std::cout<<"no cs :(((("<<std::endl;
			}
			for(auto const& child : phys_node->get_children()){
				collect_collision_shapes(&*child,collision_shapes);
			}

		}
		/*else{
			for(auto const& child : node->get_children()){
				collect_collision_shapes(&*child,collision_shapes);
			}
		}*///FALSE DONT COLLECT CS UNDER COLLIDABLE NODE
	}
	else{
		for(auto const& child : node->get_children()){
			collect_collision_shapes(&*child,collision_shapes);
		}
	}
}

void
PhysicalNode::warn_parent_physics(Node* parent)const{
	if(parent){
		//std::cout<<"check parent named: "<<parent->get_name()<<std::endl;
		//if(std::dynamic_pointer_cast<PhysicalNode>(parent)){
		if(dynamic_cast<PhysicalNode*>(parent)){
			//std::shared_ptr<PhysicalNode> phys_node = std::dynamic_pointer_cast<gua::PhysicalNode>(parent);
			auto phys_node = dynamic_cast<PhysicalNode*>(parent);
			if(phys_node->is_collidable()){
				if(!phys_node->update_physics_structure()){
					std::cout<<"Error in Update Phyics Structure of Parent!"<<std::endl;
				}
			}
			else if(parent->get_parent()){
				warn_parent_physics(parent->get_parent());
			}
		}
		else if(parent->get_parent()){
			warn_parent_physics(parent->get_parent());
		}
	}

}

bool
PhysicalNode::update_physics_structure(){
	make_collidable(false,false);
	return make_collidable(true,false);
}



}