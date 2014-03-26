// class header
#include <gua/physics/PhysicalNode.hpp>


namespace gua{


PhysicalNode::PhysicalNode(
                 std::shared_ptr<GeometryNode> const& geom,
                 physics::Physics* physics,
				 std::shared_ptr<physics::CollisionShapeNode> const& cs,
                 float mass,
                 float friction,
                 float restitution)
		:   TransformNode(geom->get_name()+"_physical"),
			rigid_body_(nullptr),
			mass_(mass),
			//cs_already_simulated_in_(nullptr),
			friction_(friction),
			restitution_(restitution),
			collision_shape_(cs),
			physics_(physics),
			geometry_(geom),
			scale_(math::vec3(1.0,1.0,1.0)),
			set_scale_(false),
			cs_collector_(new CollisionShapeCollector())
		{
			add_child(geometry_);
		}



bool
PhysicalNode::simulate(bool b_simulate,bool warn_parent){

	if(b_simulate){

		if(rigid_body_==nullptr){


			auto geom_world_ = geometry_->get_world_transform();

			//std::cout<<"geome world in physical node: "<<geom_world_<<std::endl;
			//std::cout<<"geom path in physical node: "<<geometry_->get_path()<<std::endl;


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

			
			cs_collector_->check(this);

			//create physics subgraph: (root - rb - cs)
			///root:
			auto phys_root = new TransformNode("phys_root");

			///rigidbody and collisionshape of first geometry:

			//rigidbody just consist of center of mass translation - offset in collisionshape and geometry
			if(mass_ != 0){

				math::vec3 com = cs_collector_->get_center_of_mass();


				rigid_body_= std::make_shared<physics::RigidBodyNode>(get_name()+"_rb_",mass_,friction_,restitution_,scm::math::make_translation(com));
				

				set_transform(scm::math::make_translation(com));


				collision_shape_->set_transform(scm::math::inverse(rigid_body_->get_transform()) * geom_world_* scm::math::inverse(scm::math::make_scale(scale_)));

				geometry_->set_transform(scm::math::inverse(scm::math::make_translation(com)) * geom_world_);//* scm::math::inverse(scm::math::make_scale(scale_)));// * scm::math::make_scale(scale_));

			}
			else{//static object can hold all transformations of upper geometry

				rigid_body_= std::make_shared<physics::RigidBodyNode>(get_name()+"_rb_",mass_,friction_,restitution_,geom_world_ * scm::math::inverse(scm::math::make_scale(scale_)));


				set_transform(geom_world_);


				collision_shape_->set_transform(math::mat4::identity());

				geometry_->set_transform(math::mat4::identity());
			}



			rigid_body_->add_child(collision_shape_);

			//add to physics root:
			phys_root->add_child(rigid_body_);

			cs_collector_->add_shapes_to_rb(&*rigid_body_);

			physics_->add_rigid_body(std::make_pair(rigid_body_,this));
			
			if(warn_parent)warn_parent_physics(get_parent());
			/*if(cs_already_simulated_in_){
				cs_already_simulated_in_->update_physics_structure();
				cs_already_simulated_in_ = nullptr;
			}*/
		}
		else{
			return update_physics_structure();
		}
	}
	else if(rigid_body_){
		physics_->remove_rigid_body(rigid_body_);

		//set all transformations into geometry again
		math::mat4 parent_trans;
		if(get_parent()){
			parent_trans = get_parent()->get_world_transform();
		}
		else{
			parent_trans = math::mat4::identity();
		}

		geometry_->set_transform(scm::math::inverse(parent_trans) * get_transform()  * geometry_->get_transform());
		set_transform(math::mat4::identity());

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
			set_transform(transform * scm::math::make_scale(scale_));
		}
		else{
			set_transform(transform);
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

void
PhysicalNode::calculate_collision_shape(){
	
	//std::cout<<"Attention: Trying to build CollisionShape automatically!!!"<<std::endl;

	std::string cs_name = geometry_->get_name()
					+"_automatic_collision_shape_"
					+gua::string_utils::to_string(scale_.x)+"_"
					+gua::string_utils::to_string(scale_.y)+"_"
					+gua::string_utils::to_string(scale_.z);

	auto existing_cs = gua::physics::CollisionShapeDatabase::instance()->lookup(cs_name);
	if(existing_cs == nullptr){

		std::vector<std::string> geometry_list = std::vector<std::string>();
		geometry_list.push_back(geometry_->data.get_geometry());
		auto cs = gua::physics::TriangleMeshShape::FromGeometry(geometry_list, true, true);
		cs->set_scaling(scale_);
		gua::physics::CollisionShapeDatabase::add_shape(cs_name, cs);

	}	
	std::shared_ptr<gua::physics::CollisionShapeNode> csn (new gua::physics::CollisionShapeNode(cs_name));
	csn->data.set_shape(cs_name);
	collision_shape_ = csn;
}


/*void
PhysicalNode::cs_already_simulated_in(PhysicalNode* pn){
	cs_already_simulated_in_ = pn;
}*/


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
	bool was_collidable = false;
	math::vec3 saved_angular_vel;
	math::vec3 saved_linear_vel;
	
	if(is_simulating()){
		was_collidable = true;

		saved_angular_vel = rigid_body_->angular_velocity();
		saved_linear_vel = rigid_body_->linear_velocity();

		simulate(false,false);
		collision_shape_.reset();
		collision_shape_ = nullptr;
		set_scale_ = false;
	}
	
	geometry_->scale(x,y,z);

	if(was_collidable){
		simulate(true,false);
		rigid_body_->set_angular_velocity(saved_angular_vel);
		rigid_body_->set_linear_velocity(saved_linear_vel);
	}
}

void
PhysicalNode::rotate(float angle, float x, float y, float z){
	bool was_collidable = false;
	math::vec3 saved_angular_vel;
	math::vec3 saved_linear_vel;

	if(is_simulating()){
		was_collidable = true;

		saved_angular_vel = rigid_body_->angular_velocity();
		saved_linear_vel = rigid_body_->linear_velocity();

		simulate(false,false);
	}
	
	geometry_->rotate(angle, x, y, z);

	if(was_collidable){
		simulate(true,false);
		rigid_body_->set_angular_velocity(saved_angular_vel);
		rigid_body_->set_linear_velocity(saved_linear_vel);
	}
}

void
PhysicalNode::translate(float x, float y, float z){
	bool was_collidable = false;
	math::vec3 saved_angular_vel;
	math::vec3 saved_linear_vel;

	if(is_simulating()){
		was_collidable = true;

		saved_angular_vel = rigid_body_->angular_velocity();
		saved_linear_vel = rigid_body_->linear_velocity();

		simulate(false,false);
	}
	
	geometry_->translate(x,y,z);

	if(was_collidable){
		simulate(true,false);
		rigid_body_->set_angular_velocity(saved_angular_vel);
		rigid_body_->set_linear_velocity(saved_linear_vel);
	}
}

/*//RigidBody Interface
////////////////////////////////////////////////////////////////////////////////

void PhysicalNode::set_kinematic(bool kinematic) {rigid_body_->set_kinematic(kinematic);}

////////////////////////////////////////////////////////////////////////////////

void PhysicalNode::set_mass(float mass) {rigid_body_->set_mass(mass);}

////////////////////////////////////////////////////////////////////////////////

void PhysicalNode::set_friction(float frict) {rigid_body_->set_friction(frict);}

////////////////////////////////////////////////////////////////////////////////

void PhysicalNode::set_rolling_friction(float frict) {rigid_body_->set_rolling_friction(frict);}

////////////////////////////////////////////////////////////////////////////////

void PhysicalNode::set_restitution(float rest) {rigid_body_->set_restitution(rest);}

////////////////////////////////////////////////////////////////////////////////

void PhysicalNode::set_damping(float lin_damping, float ang_damping) {rigid_body_->set_damping(lin_damping,ang_damping);}

////////////////////////////////////////////////////////////////////////////////

void PhysicalNode::apply_force(const math::vec3& force,
                                const math::vec3& rel_pos) {rigid_body_->apply_force(force,rel_pos);}

////////////////////////////////////////////////////////////////////////////////

void PhysicalNode::apply_central_force(const math::vec3& force) {rigid_body_->apply_central_force(force);}

////////////////////////////////////////////////////////////////////////////////

void PhysicalNode::apply_torque(const math::vec3& torque) {rigid_body_->apply_torque(torque);}

////////////////////////////////////////////////////////////////////////////////

void PhysicalNode::apply_torque_impulse(const math::vec3& torque) {rigid_body_->apply_torque_impulse(torque);}

////////////////////////////////////////////////////////////////////////////////

void PhysicalNode::apply_impulse(const math::vec3& impulse,
                                  const math::vec3& rel_pos) {rigid_body_->apply_impulse(impulse,rel_pos);}

////////////////////////////////////////////////////////////////////////////////

void PhysicalNode::apply_central_impulse(const math::vec3& impulse) {rigid_body_->apply_central_impulse(impulse);}

////////////////////////////////////////////////////////////////////////////////

void PhysicalNode::clear_forces() {rigid_body_->clear_forces();}

////////////////////////////////////////////////////////////////////////////////

void PhysicalNode::set_angular_velocity(const math::vec3& vel) {rigid_body_->set_angular_velocity(vel);}

////////////////////////////////////////////////////////////////////////////////

math::vec3 PhysicalNode::angular_velocity() const {return rigid_body_->angular_velocity();}

////////////////////////////////////////////////////////////////////////////////

void PhysicalNode::set_linear_velocity(const math::vec3& vel) {rigid_body_->set_linear_velocity(vel);}

////////////////////////////////////////////////////////////////////////////////

math::vec3 PhysicalNode::linear_velocity() const {return rigid_body_->linear_velocity();}*/



}