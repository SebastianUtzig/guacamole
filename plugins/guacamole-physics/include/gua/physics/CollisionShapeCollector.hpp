#ifndef GUA_COLLISION_SHAPE_COLLECTOR_HPP
#define GUA_COLLISION_SHAPE_COLLECTOR_HPP



namespace gua{

class PhysicalNode;

class CollisionShapeCollector : public NodeVisitor {
 
 public:

 	CollisionShapeCollector();

 	virtual ~CollisionShapeCollector();


 	void 		check(PhysicalNode* pn);

 	void 		visit(PhysicalNode* node);

 	math::vec3	get_center_of_mass()const;

 	void 		add_shapes_to_rb(physics::RigidBodyNode* rb);





 private:

 	std::list<std::tuple<std::shared_ptr<physics::CollisionShapeNode>,math::mat4,float>> collected_collision_shapes_;



};


}

#endif