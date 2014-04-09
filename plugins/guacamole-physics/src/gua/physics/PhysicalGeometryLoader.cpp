// class header
#include <gua/physics/PhysicalGeometryLoader.hpp>


namespace gua{


PhysicalGeometryLoader::PhysicalGeometryLoader(physics::Physics * ph)
		:physics_(ph),
		 mass_(1.0){}

PhysicalGeometryLoader::~PhysicalGeometryLoader(){}


void
PhysicalGeometryLoader::visit(GeometryNode* geom){

	//put physical node in between geom and parent
	auto parent = geom->get_parent();

	std::shared_ptr<GeometryNode>shared_geom(geom);

	auto children = parent->get_children();
	for (auto c(children.begin()); c != children.end(); ++c) {
        if (*c == shared_geom) {
            children.erase(c);
            //shared_geom->parent_ = nullptr;
            //set_dirty();
            break;
        }
    }


	auto physical_node = new gua::PhysicalNode(shared_geom,physics_,nullptr,mass_);

	std::shared_ptr<Node>shared_node(physical_node);

	parent->add_child(shared_node);

	for(auto child : geom->get_children()){
		child->accept(*this);
	}


}

void
PhysicalGeometryLoader::visit(Node* node){

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

std::shared_ptr<PhysicalNode>
PhysicalGeometryLoader::create_physical_objects_from_file(
						  std::string const& node_name,
                          std::string const& file_name,
                          std::string const& fallback_material,
                          float mass,
                          std::shared_ptr<physics::CollisionShapeNode> const& collision_shape
                          )
{
	auto node = loader_.create_geometry_from_file(node_name, file_name, fallback_material);


	if(node){
		if(node->get_children().size() == 0){

			std::shared_ptr<gua::GeometryNode> geometry = std::dynamic_pointer_cast<gua::GeometryNode>(node);
			if(geometry){

				auto physical_node = new gua::PhysicalNode(geometry,physics_,collision_shape,mass);

				std::shared_ptr<PhysicalNode>return_node(physical_node);
				
				return return_node;
			}
			else{
				std::cout<<"Error in PhysicalGeometryLoader: Single loaded node isnt a GeometryNode!"<<std::endl;
			}

		}
		else{
			//geometry group
			mass_ = mass;

			auto list = node->get_children();

			for(auto child: list){
				child->accept(*this);
			}

			auto physical_node = new gua::PhysicalNode(node,physics_,collision_shape,mass);

			std::shared_ptr<PhysicalNode>return_node(physical_node); 
			return return_node;
		}

	}

}



}
