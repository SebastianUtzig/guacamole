#include <gua/physics/CollisionShapeCollector.hpp>
#include <gua/physics/PhysicalNode.hpp>


namespace gua{

CollisionShapeCollector::CollisionShapeCollector():
	current_root_(nullptr),
	collected_collision_shapes_(std::list<std::tuple<std::shared_ptr<physics::CollisionShapeNode>,math::mat4,float>>())
	{}

CollisionShapeCollector::~CollisionShapeCollector(){
	while(!collected_collision_shapes_.empty()){
		collected_collision_shapes_.pop_back();
	}


}


void
CollisionShapeCollector::check(PhysicalNode* pn){

	collected_collision_shapes_.clear();

	current_root_ = pn;

	pn->accept(*this);

}

void
CollisionShapeCollector::visit(PhysicalNode* node){
	if(!node->is_simulating()){
		/*auto cs = node->get_collision_shape();
		if(cs){	
			
			auto transform = node->get_geometry()->get_world_transform();

			collected_collision_shapes_.push_back(std::make_tuple(cs,transform,node->get_mass()));

		 	//node->cs_already_simulated_in(current_root_);
		}
		else{
			std::cout<<"no cs :(((("<<std::endl;
		}
		for(auto const& child : node->get_children()){
			child->accept(*this);
		}*/
		for(auto const& child : node->get_children()){
			child->accept(*this);
		}
	}

}

void
CollisionShapeCollector::visit(Node* node){
	/*for(auto child : node->get_children()){
		child->accept(*this);
	}*/
	auto geom = dynamic_cast<GeometryNode*>(node);
	if(geom){
		visit(geom);
	}
	else{
		auto list = node->get_children();
		for(auto child : list){
			child->accept(*this);
		}
	}
}

void
CollisionShapeCollector::visit(GeometryNode* geom){

	//just simulate if parent is a physicalnode
	auto phys_node =dynamic_cast<PhysicalNode*>(geom->get_parent());
	if(phys_node){

	//	std::cout<<"will check world transform of : "<<geom->get_path()<<std::endl;

		auto geom_world = geom->get_world_transform();

	//	std::cout<<"found geom with world: "<<geom_world<<std::endl;

		//getScale solution from avango-gua
		math::vec3 x_vec(geom_world[0], geom_world[1], geom_world[2]);
		math::vec3 y_vec(geom_world[4], geom_world[5], geom_world[6]);
		math::vec3 z_vec(geom_world[8], geom_world[9], geom_world[10]);
		auto scale = math::vec3(scm::math::length(x_vec), scm::math::length(y_vec), scm::math::length(z_vec));

		//std::cout<<"Attention: Trying to build CollisionShape automatically!!!"<<std::endl;

		std::string cs_name = geom->get_name()
						+"_automatic_collision_shape_"
						+gua::string_utils::to_string(scale.x)+"_"
						+gua::string_utils::to_string(scale.y)+"_"
						+gua::string_utils::to_string(scale.z);

		auto existing_cs = gua::physics::CollisionShapeDatabase::instance()->lookup(cs_name);
		if(existing_cs == nullptr){

			std::vector<std::string> geometry_list = std::vector<std::string>();
			geometry_list.push_back(geom->data.get_geometry());
			auto cs = gua::physics::TriangleMeshShape::FromGeometry(geometry_list, true, true);
			cs->set_scaling(scale);
			gua::physics::CollisionShapeDatabase::add_shape(cs_name, cs);

		}	
		std::shared_ptr<gua::physics::CollisionShapeNode> csn (new gua::physics::CollisionShapeNode(cs_name));
		csn->data.set_shape(cs_name);

		//collision_shape is csn;

		collected_collision_shapes_.push_back(std::make_tuple(csn,geom_world,phys_node->get_mass()));

	}

	for(auto child : geom->get_children()){
		child->accept(*this);
	}
//	std::cout<<"after cs calculation"<<std::endl;

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

	if(new_mass > 0.0){
		return com / new_mass;
	}
	else{
		//std::cout<<" Attention: No collision shapes found in CollisionShapeCollector!"<<std::endl;//or just static ones
		return com;
	}
}

void
CollisionShapeCollector::add_shapes_to_rb(physics::RigidBodyNode* rb){
	//std::cout<<"collected_collision_shapes_ length: "<<collected_collision_shapes_.size()<<std::endl;
	for(auto cs : collected_collision_shapes_){

		auto cs_world = std::get<1>(cs);

		//getScale solution from avango-gua
		math::vec3 x_vec(cs_world[0], cs_world[1], cs_world[2]);
	    math::vec3 y_vec(cs_world[4], cs_world[5], cs_world[6]);
	    math::vec3 z_vec(cs_world[8], cs_world[9], cs_world[10]);
	    auto cs_scale = math::vec3(scm::math::length(x_vec), scm::math::length(y_vec), scm::math::length(z_vec));

		//similar to transformation of own cs:
		std::get<0>(cs)->set_transform(scm::math::inverse(rb->get_transform()) * cs_world * scm::math::inverse(scm::math::make_scale(cs_scale)));//all other collision shapes in lower graph must be in rb coord.syst.

		rb->add_child(std::get<0>(cs));
	}
}


}
