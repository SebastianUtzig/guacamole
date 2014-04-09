// class header
#include <gua/physics/PhysicalNode.hpp>


namespace gua{


PhysicalNode::PhysicalNode(
                 std::shared_ptr<Node> const& child,
                 physics::Physics* physics,
				 std::shared_ptr<physics::CollisionShapeNode> const& cs,
                 float mass,
                 float friction,
                 float restitution)
		:   TransformNode(child->get_name()+"_physical"),
			rigid_body_(nullptr),
			mass_(mass),
			friction_(friction),
			restitution_(restitution),
			collision_shape_(cs),
			physics_(physics),
			center_of_mass_(math::vec3(0.0,0.0,0.0)),
			child_(child),
			scale_(math::vec3(1.0,1.0,1.0)),
			set_scale_(false),
			cs_collector_(new CollisionShapeCollector())
		{
			if(child->get_parent()){
				//put new physical node in between child and his parent

				auto parent = child->get_parent();

				auto children = parent->get_children();
				for (auto c(children.begin()); c != children.end(); ++c) {
			        if (*c == child) {
			            children.erase(c);
			            //shared_geom->parent_ = nullptr;
			            //set_dirty();
			            break;
			        }
			    }
			}
			add_child(child);
		}



bool
PhysicalNode::simulate(bool b_simulate,bool warn_parent){

	if(b_simulate){

		if(rigid_body_==nullptr){
			
			cs_collector_->check(this);


			//create physics subgraph: (root - rb - cs)
			///root:
			auto phys_root = new TransformNode("phys_root");

			//rigidbody just consist of center of mass translation - offset in collisionshape
			if(mass_ != 0){

				center_of_mass_ = cs_collector_->get_center_of_mass();
		
				for(auto child : get_children()){
					auto child_world = child->get_world_transform();
					child->set_transform(child_world);
				}
				set_transform(math::mat4::identity());



				rigid_body_= std::make_shared<physics::RigidBodyNode>(get_name()+"_rb_",mass_,friction_,restitution_,scm::math::make_translation(center_of_mass_));

			}
			else{//static object can hold all transformations of upper geometry

				auto geom = std::dynamic_pointer_cast<GeometryNode>(child_);
				if(geom){
					//getScale solution from avango-gua
					auto geom_world = geom->get_world_transform();
					math::vec3 x_vec(geom_world[0], geom_world[1], geom_world[2]);
				    math::vec3 y_vec(geom_world[4], geom_world[5], geom_world[6]);
				    math::vec3 z_vec(geom_world[8], geom_world[9], geom_world[10]);
				    auto scale = math::vec3(scm::math::length(x_vec), scm::math::length(y_vec), scm::math::length(z_vec));

					for(auto child : get_children()){
						auto child_world = child->get_world_transform();
						child->set_transform(child_world);
					}
					set_transform(math::mat4::identity());
					rigid_body_= std::make_shared<physics::RigidBodyNode>(get_name()+"_rb_",mass_,friction_,restitution_,geom_world * scm::math::inverse(scm::math::make_scale(scale)));

				}
				else{

					for(auto child : get_children()){
						auto child_world = child->get_world_transform();
						child->set_transform(child_world);
					}

					rigid_body_= std::make_shared<physics::RigidBodyNode>(get_name()+"_rb_",mass_,friction_,restitution_,math::mat4::identity());
					set_transform(math::mat4::identity());
				}

			}


			//add to physics root:
			phys_root->add_child(rigid_body_);

			cs_collector_->add_shapes_to_rb(&*rigid_body_);

			physics_->add_rigid_body(std::make_pair(rigid_body_,this));
			
			if(warn_parent)warn_parent_physics(get_parent());

		}
		else{
			return update_physics_structure();
		}
	}
	else if(rigid_body_){

		math::mat4 parent_trans;
		if(get_parent()){
			parent_trans = get_parent()->get_world_transform();
		}
		else{
			parent_trans = math::mat4::identity();
		}

		for(auto child : get_children()){
			auto child_trans = child->get_transform();
			child->set_transform(scm::math::inverse(parent_trans) * get_transform()  * child_trans);
		}

		set_transform(math::mat4::identity());

		
		physics_->remove_rigid_body(rigid_body_);

		rigid_body_.reset();

		rigid_body_ = nullptr;
		if(warn_parent)warn_parent_physics(get_parent());
	}

	return true;

}


bool
PhysicalNode::is_simulating()const{
	if (rigid_body_){
		return true;
	}
	return false;
}


void
PhysicalNode::set_world_transform(math::mat4 const& transform){
	if(is_simulating()){
		if(mass_ == 0){
			set_transform(math::mat4::identity());//doesnt make sense, but simulation will stop without it (???)
		}
		else{
			set_transform(transform * scm::math::inverse(scm::math::make_translation(center_of_mass_)));
		}
	}
	else{
		world_transform_ = transform;//sinnvoll?
	}
}

math::mat4
PhysicalNode::get_world_transform()const{
	if(!is_simulating()){
		if (parent_){
	        return parent_->get_world_transform() * get_transform();
	    }
	    return get_transform();
	}
	else{
    	return get_transform();
	}
}

std::shared_ptr<physics::CollisionShapeNode>
PhysicalNode::get_collision_shape(){
	return collision_shape_;
}


std::shared_ptr<physics::RigidBodyNode>
PhysicalNode::get_rigid_body()const{
	return rigid_body_;
}

float
PhysicalNode::get_mass()const{
	return mass_;
}
void
PhysicalNode::set_mass(float mass){
	mass_ = mass;
	
	if(is_simulating()){
		if(mass_ >0.0 && mass > 0.0){
			rigid_body_->set_mass(mass);
		}
		else{
			// from static to dynamic or other way around
			simulate(false,false);
			simulate(true,false);
		}
	}
}



void
PhysicalNode::update_cache(){
	if(!is_simulating()){
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
PhysicalNode::warn_parent_physics(Node* parent)const{
	if(parent){
		if(dynamic_cast<PhysicalNode*>(parent)){
			auto phys_node = dynamic_cast<PhysicalNode*>(parent);
			if(phys_node->is_simulating()){
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
	simulate(false,false);
	return simulate(true,false);
}

void
PhysicalNode::accept(NodeVisitor& visitor) {

	auto collector = dynamic_cast<CollisionShapeCollector*>(&visitor);
	if(collector){
		collector->visit(this);
	}
	else{
		visitor.visit(this);
	}

}

void
PhysicalNode::scale(float x, float y, float z){
	bool was_simulating = false;
	math::vec3 saved_angular_vel;
	math::vec3 saved_linear_vel;
	
	if(is_simulating()){
		was_simulating = true;

		saved_angular_vel = rigid_body_->angular_velocity();
		saved_linear_vel = rigid_body_->linear_velocity();

		simulate(false,false);
		collision_shape_.reset();
		collision_shape_ = nullptr;
		set_scale_ = false;
	}
	
	set_transform(scm::math::make_scale(x, y, z) * get_transform());
	set_dirty();

	if(was_simulating){
		simulate(true,false);
		rigid_body_->set_angular_velocity(saved_angular_vel);
		rigid_body_->set_linear_velocity(saved_linear_vel);
	}
}

void
PhysicalNode::rotate(float angle, float x, float y, float z){
	bool was_simulating = false;
	math::vec3 saved_angular_vel;
	math::vec3 saved_linear_vel;

	if(is_simulating()){
		was_simulating = true;

		saved_angular_vel = rigid_body_->angular_velocity();
		saved_linear_vel = rigid_body_->linear_velocity();

		simulate(false,false);
	}
	
	set_transform(scm::math::make_rotation(angle, x, y, z) * get_transform());
	set_dirty();

	if(was_simulating){
		simulate(true,false);
		rigid_body_->set_angular_velocity(saved_angular_vel);
		rigid_body_->set_linear_velocity(saved_linear_vel);
	}
}

void
PhysicalNode::translate(float x, float y, float z){
	bool was_simulating = false;
	math::vec3 saved_angular_vel;
	math::vec3 saved_linear_vel;

	if(is_simulating()){
		was_simulating = true;

		saved_angular_vel = rigid_body_->angular_velocity();
		saved_linear_vel = rigid_body_->linear_velocity();

		simulate(false,false);
	}
	
	set_transform(scm::math::make_translation(x, y, z) * get_transform());
	set_dirty();

	if(was_simulating){
		simulate(true,false);
		rigid_body_->set_angular_velocity(saved_angular_vel);
		rigid_body_->set_linear_velocity(saved_linear_vel);
	}
}


}