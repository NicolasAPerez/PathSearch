#include "PathSearch.h"


namespace ufl_cap4053
{
	namespace searches
	{
		
		
		PathSearch::PathSearch() : toVisit{greaterThan}
		{
			target = nullptr;
			visiting = nullptr;
			map = nullptr;
		}
		PathSearch::~PathSearch()
		{
			shutdown();
			unload();
		}
		void PathSearch::load(TileMap* _tileMap)
		{
			map = _tileMap;
			visitMap.resize(map->getColumnCount(), std::vector<bool>(map->getRowCount()));
			costMap.resize(map->getColumnCount(), std::vector<Node*>(map->getRowCount()));
		}
		void PathSearch::initialize(int startRow, int startCol, int goalRow, int goalCol)
		{
			//Create new Node and add it to visit list
			toVisit.push(new Node(map->getTile(startRow, startCol), nullptr ,0));

			memHolder.push_back(toVisit.front());
			visitMap[startCol][startRow] = true;
			//Set target tile
			target = map->getTile(goalRow, goalCol);



		}
		void PathSearch::update(long timeslice)
		{
			//Set up timer
			auto startTime = std::chrono::steady_clock::now();
			auto endTime = startTime;
			std::chrono::duration<double, std::milli> dur = endTime - startTime;

			while (dur.count() <= timeslice && !isDone()) {
				
				//Set which node is currently visiting
				visiting = (toVisit.front()->tile);
				tileVisitOrder.push_back(toVisit.front());
				toVisit.front()->tile->setFill(0xFF0000FF);

				toVisit.pop();

				switch (modeSel) {
					case 'G':
						greedyBestFirst();
						break;
					case 'U':
						uniformCost();
						break;
					case 'A':
						aStar(1);
						break;
				}


				//Recalc time before while loop
				endTime = std::chrono::steady_clock::now();
				dur = endTime - startTime;
			}
		}
		void PathSearch::shutdown()
		{
			//Previous implementaion, lead to crt is vaild heap pointer errors
			/*
			int visitSize = toVisit.size();
			for (int i = 0; i < visitSize; i++) {
				delete toVisit.front();
				toVisit.pop();
			}
			toVisit.clear();

			int visitOrderSize = tileVisitOrder.size();
			for (int i = 0; i < visitOrderSize; i++) {
				delete tileVisitOrder[i];
			}
			*/
			for (int i = 0; i < visitMap.size(); i++) {
				for (int j = 0; j < visitMap.at(i).size(); j++) {
					visitMap.at(i).at(j) = false;
					costMap.at(i).at(j) = nullptr;
				}
			}
			

			
			size_t memHolderSize = memHolder.size();
			for (int i = 0; i < memHolderSize; i++) {
				delete memHolder[i];
			}

			toVisit.clear();
			memHolder.clear();
			tileVisitOrder.clear();
			visiting = nullptr;
			target = nullptr;

			
		}
		void PathSearch::unload()
		{
			map = nullptr;
			visitMap.clear();
			costMap.clear();
			
		}
		bool PathSearch::isDone() const
		{
			return visiting == target;
		}
		std::vector<Tile const*> const PathSearch::getSolution() const
		{
			std::vector<Tile const*> path;
			if (!tileVisitOrder.empty()){
				tileVisitOrder.back()->showPath(path);
				
			}
			std::vector<Tile const*> const pathResult = path;
			std::cout << path.back()->getColumn() << path.back()->getRow();
			
			return pathResult;
		}

		bool PathSearch::greaterThan(Node* const& lhs, Node* const& rhs)
		{
			bool rst;

			if (modeSel == 'G') {
				rst = lhs->distanceToGoal > rhs->distanceToGoal;
			}
			else if (modeSel == 'U') {
				rst = lhs->costToSource > rhs->costToSource;
			}
			else if (modeSel == 'A') {
				rst = lhs->costToSource + lhs->distanceToGoal > rhs->costToSource + rhs->distanceToGoal;
			}
			return rst;
		}

		


		
		//Helper method
		bool PathSearch::areAdjacent(const Tile* lhs, const Tile* rhs)
		{
			int columnSel = (lhs->getRow() % 2 == 0)? -1 : 1;
			//Determine whether it's exactly one tile away in any of the cardinal directions
			int NSDistance = rhs->getRow() - lhs-> getRow();
			int EWDistance = rhs->getColumn() - lhs->getColumn();
			int cardinalDistance = abs(NSDistance) + abs(EWDistance);
			bool result = cardinalDistance <= 1 || (abs(NSDistance) == 1 && EWDistance == columnSel);

			return result;
			
		}

		std::vector<Tile*> PathSearch::neighboringTiles(bool visitNeighbors, bool addToCostMap, Tile* center)
		{
			std::vector<Tile*> neighbors;
			int centerCol = center->getColumn();
			int centerRow = center->getRow();
			for (int i = centerCol - 1; i < centerCol + 2; i++) {
				for (int j = centerRow - 1; j < centerRow + 2; j++) {
					//Check for vaild position
					if (i >= 0 && i < map->getColumnCount() && j >= 0 && j < map->getRowCount()) {
						//Check to see if already visited
						if (!visitMap[i][j]) {
							Tile* temp = map->getTile(j, i);
							//Finally check to see if it's adjecent and a valid tile if so put it on the list
							if (areAdjacent(center, temp) && temp->getWeight() != 0) {
								visitMap[i][j] =  (visitNeighbors)? true : visitMap[i][j];
								neighbors.push_back(temp);

							}
						}
					}
				}
			}

			return neighbors;
		}

		//Algortihm Methods
		void PathSearch::greedyBestFirst()
		{
			
			if (isDone()) {
				return;
			}

			std::vector<Tile*> neighbors = neighboringTiles(true, false, visiting);
			for (Tile* t : neighbors){

				Node* next = new Node(t, tileVisitOrder.back(), tileVisitOrder.back()->costToSource + t->getWeight());
				next->distanceToGoal = sqrt(pow(t->getXCoordinate() - target->getXCoordinate(),2) + pow(t->getYCoordinate() - target->getYCoordinate(), 2));
				toVisit.push(next);
				memHolder.push_back(next);

				//DEBUG:
				t->setMarker(0xFF00FF00);

			}

		}

		void PathSearch::uniformCost()
		{
			if (isDone()) {
				return;
			}

			std::vector<Tile*> neighbors = neighboringTiles(true, false, visiting);
			for (Tile* t : neighbors) {
				
				int i = t->getColumn();
				int j = t->getRow();
				Node* next;

				//Check to see if node is already "estimated"
				if (costMap[i][j] == nullptr){
					next = new Node(t, tileVisitOrder.back(), tileVisitOrder.back()->costToSource + t->getWeight());
					
					toVisit.push(next);
					costMap[i][j] = next;
					memHolder.push_back(next);
				}
				else {
					next = costMap[i][j];
					int cost = tileVisitOrder.back()->costToSource + t->getWeight();

					if (next->costToSource > cost) {
						next->costToSource = cost;
						next->prevTile = tileVisitOrder.back();
					}
				}
				//DEBUG:
				t->setMarker(0xFF00FF00);

				if (t == target) {
					tileVisitOrder.push_back(next);
					visiting = target;
					return;
				}
			}
		}

		void PathSearch::aStar(double hWeight)
		{
			if (isDone()) {
				return;
			}

			std::vector<Tile*> neighbors = neighboringTiles(true, false, visiting);
			for (Tile* t : neighbors) {

				int i = t->getColumn();
				int j = t->getRow();
				Node* next;

				//Check to see if node is already "estimated"
				if (costMap[i][j] == nullptr) {
					double aStarGivenCost = tileVisitOrder.back()->costToSource + (t->getWeight() * sqrt(pow(t->getXCoordinate() - visiting->getXCoordinate(), 2) + pow(t->getYCoordinate() - visiting->getYCoordinate(), 2)));
					next = new Node(t, tileVisitOrder.back(), aStarGivenCost);
					next->distanceToGoal = hWeight * sqrt(pow(t->getXCoordinate() - target->getXCoordinate(), 2) + pow(t->getYCoordinate() - target->getYCoordinate(), 2));
					toVisit.push(next);
					memHolder.push_back(next);
					costMap[i][j] = next;
				}
				else {
					next = costMap[i][j];
					int cost = tileVisitOrder.back()->costToSource + t->getWeight();

					if (next->costToSource > cost) {
						next->costToSource = cost;
						next->prevTile = tileVisitOrder.back();
					}
				}
				//DEBUG:
				t->setMarker(0xFF55FF55);

				if (t == target) {
					tileVisitOrder.push_back(next);
					visiting = target;
					return;
				}
			}
		}


		PathSearch::Node::Node(Tile* tile, Node* prevTile, int costToSource)
		{
			this->tile = tile;
			this->prevTile = prevTile;
			this->costToSource = costToSource;
			distanceToGoal = 0;
		}

		PathSearch::Node::~Node()
		{
		}

		void PathSearch::Node::showPath(std::vector<Tile const*> &path) const
		{
			
			if (prevTile == nullptr) {
				path.insert(path.begin(), tile);
			}
			else {
				prevTile->showPath(path);
				path.insert(path.begin(), tile);
			}
			this->tile->setFill(0xFFFF0000);
		}
		
	}
}  // close namespace ufl_cap4053::searches
