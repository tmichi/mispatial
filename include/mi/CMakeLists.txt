SET (include_files #BoundingBox.hpp
		   Kdtree.hpp
		   Octree.hpp
		   OctreeExporter.hpp
		   OctreeImporter.hpp
		)
INSTALL(FILES ${include_files}
        DESTINATION include/mi
        PERMISSIONS OWNER_READ GROUP_READ WORLD_READ)
