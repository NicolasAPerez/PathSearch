#include <chrono>
#include <iostream>
#include <cmath>

#include "../platform.h" // This file will make exporting DLL symbols simpler for students.
#include "../Framework/TileSystem/TileMap.h"
#include "../PriorityQueue.h"



namespace ufl_cap4053
{
	namespace searches
	{
		class PathSearch
		{
		// CLASS DECLARATION GOES HERE
			public:
				//PathSearch();
				DLLEXPORT PathSearch(); // EX: DLLEXPORT required for public methods - see platform.h
				DLLEXPORT ~PathSearch();
				DLLEXPORT void load(TileMap* _tileMap);
				DLLEXPORT void initialize(int startRow, int startCol, int goalRow, int goalCol);
				DLLEXPORT void update(long timeslice);
				DLLEXPORT void shutdown();
				DLLEXPORT void unload();
				DLLEXPORT bool isDone() const;
				DLLEXPORT std::vector<Tile const*> const getSolution() const;

			private:

				//Select between the different algorithm types
				//'G' for Greedy, 'U' for Uniform Cost, 'A' for A*
				static const char modeSel = 'A';
				//Helper Node Class
				class Node
				{
				public:
					//Note: Originally designed this node for info needed in Dijstraka's(butchered the name) algorithm
					Tile* tile;
					Node* prevTile;
					int costToSource;
					double distanceToGoal;
					Node(Tile* tile, Node* prevTile, int costToSource);
					~Node();
					void showPath(std::vector<Tile const*> &path) const;
					
				};

				//Static GreaterThan method for PrioityQueue
				static bool greaterThan(Node* const& lhs, Node* const& rhs);
				

				//Variables
				PriorityQueue<Node*> toVisit;
				TileMap* map;
				Tile* visiting;
				Tile* target;
				std::vector<std::vector<bool>> visitMap;
				std::vector<std::vector<Node*>> costMap;
				

				

				std::vector<Node*> tileVisitOrder;

				//Note: running into errors where pointers are somehow deleted twice, "duct tape" fix but hopefully works
				std::vector<Node*> memHolder;


				bool  areAdjacent(const Tile* lhs, const Tile* rhs);
				std::vector<Tile*> neighboringTiles(bool visitNeighbors, bool addToCostMap, Tile* center);

				//Algorithm Methods

				void greedyBestFirst();
				void uniformCost();
				void aStar(double hWeight);

		};
	}
}  // close namespace ufl_cap4053::searches
