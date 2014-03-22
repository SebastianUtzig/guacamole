// class header
#include <gua/physics/PhysicalGeometryLoader.hpp>


namespace gua{


PhysicalGeometryLoader::PhysicalGeometryLoader(physics::Physics * ph)
		:physics_(ph),
		 make_all_collidable_(false){}

PhysicalGeometryLoader::~PhysicalGeometryLoader(){}


void
PhysicalGeometryLoader::visit(GeometryNode* geom){
	


	//put physical node in between geom and parent
	auto parent = geom->get_parent();

	std::shared_ptr<GeometryNode>shared_geom(geom);

	//parent->clear_children();
	//parent->remove_child(shared_geom);
	auto children = parent->get_children();
	for (auto c(children.begin()); c != children.end(); ++c) {
        if (*c == shared_geom) {
            children.erase(c);
            //shared_geom->parent_ = nullptr;
            //set_dirty();

            break;
        }
    }


	auto physical_node = new gua::PhysicalNode(shared_geom,physics_,nullptr,0.0);
//	physical_node->scale(0.008,0.008,0.008);
//	physical_node->rotate(-90.0,1,0,0);
	physical_node->translate(0.0,0.0,-10.0);

	if(make_all_collidable_){
		physical_node->make_collidable(true);
	}

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

std::shared_ptr<Node>
PhysicalGeometryLoader::create_physical_objects_from_file(
						  std::string const& node_name,
                          std::string const& file_name,
                          std::string const& fallback_material,
                          bool make_collidable
                          )
{
	auto node = loader_.create_geometry_from_file(node_name, file_name, fallback_material);


	if(node){
		if(node->get_children().size() == 0){

			std::shared_ptr<gua::GeometryNode> geometry = std::dynamic_pointer_cast<gua::GeometryNode>(node);
			if(geometry){
				//geometry->data.set_geometry("");
				//auto phys_node2 = new gua::PhysicalNode(geometry,&physics,csn2);
				auto physical_node = new gua::PhysicalNode(geometry,physics_,nullptr,1.0);

				if(make_collidable){
					physical_node->make_collidable(true);
				}

				std::shared_ptr<Node>return_node(physical_node);
				
				
				return return_node;
			}
			else{
				std::cout<<"Error in PhysicalGeometryLoader: Single loaded node isnt a GeometryNode!"<<std::endl;
			}

		}
		else{
			//geometry group
			make_all_collidable_ = make_collidable;

			auto list = node->get_children();

			for(auto child: list){
				child->accept(*this);
			}

			return node;
		}

	}

}





}
