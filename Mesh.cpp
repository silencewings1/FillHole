#include <list>
#include <cstring>
#include <iostream>
#include <fstream>
#include <cmath>
#include "Mesh.h"


inline Eigen::Vector3d MinVector3d(Eigen::Vector3d v1, Eigen::Vector3d v2) {
	return Eigen::Vector3d(std::min(v1(0), v2(0)),
		std::min(v1(1), v2(1)),
		std::min(v1(2), v2(2)));
}

inline Eigen::Vector3d MaxVector3d(Eigen::Vector3d v1, Eigen::Vector3d v2) {
	return Eigen::Vector3d(std::max(v1(0), v2(0)),
		std::max(v1(1), v2(1)),
		std::max(v1(2), v2(2)));
}

OneRingHEdge::OneRingHEdge(const Vertex* v) {
	if (v == NULL) start = next = NULL;
	else start = next = v->HalfEdge();
}

HEdge* OneRingHEdge::NextHEdge() {
	HEdge* ret = next;
	if (next && next->Prev()->Twin() != start)
		next = next->Prev()->Twin();
	else
		next = NULL;
	return ret;
}

Mesh::~Mesh() {
	Clear();
}

const HEdgeList& Mesh::Edges() const {
	return heList;
}

const HEdgeList& Mesh::BoundaryEdges() const {
	return bheList;
}

const VertexList& Mesh::Vertices() const {
	return vList;
}

const FaceList& Mesh::Faces() const {
	return fList;
}

// load a .obj mesh definition file
bool Mesh::LoadObjFile(const char* filename) {
	if (filename == NULL || strlen(filename) == 0) return false;
	std::ifstream ifs(filename);
	if (ifs.fail()) return false;

	Clear();

	std::string line;
	while (std::getline(ifs, line)) {
		if (line.empty()) continue;

		std::istringstream iss(line);
		std::string type;
		iss >> type;
		// vertex
		if (type.compare("v") == 0) {
			double x, y, z;
			iss >> x >> y >> z;
			AddVertex(new Vertex(x, y, z));
		}
		// face
		else if (type.compare("f") == 0) {
			int index[3];
			iss >> index[0] >> index[1] >> index[2];
			AddFace(index[0] - 1, index[1] - 1, index[2] - 1);
		}
	}
	ifs.close();

	size_t i;
	Eigen::Vector3d box = this->MaxCoord() - this->MinCoord();
	for (i = 0; i < vList.size(); i++) vList[i]->SetPosition(vList[i]->Position() / box(0));

	Eigen::Vector3d tot = Eigen::Vector3d::Zero();
	for (i = 0; i < vList.size(); i++) tot += vList[i]->Position();
	Eigen::Vector3d avg = tot / vList.size();
	for (i = 0; i < vList.size(); i++) vList[i]->SetPosition(vList[i]->Position() - avg);

	HEdgeList list;
	for (i = 0; i < bheList.size(); i++)
		if (bheList[i]->Start()) list.push_back(bheList[i]);
	bheList = list;

	for (i = 0; i < vList.size(); i++) {
		vList[i]->adjHEdges.clear();
		vList[i]->SetIndex((int)i);
		vList[i]->SetFlag(0);
	}

	return true;
}

void Mesh::AddVertex(Vertex* v) {
	vList.push_back(v);
}

void Mesh::AddFace(int v1, int v2, int v3) {
	int i;
	HEdge *he[3], *bhe[3];
	Vertex* v[3];
	Face* f;

	// obtain objects
	for (i = 0; i < 3; i++) he[i] = new HEdge();
	for (i = 0; i < 3; i++) bhe[i] = new HEdge(true);
	v[0] = vList[v1];
	v[1] = vList[v2];
	v[2] = vList[v3];
	f = new Face();

	// connect prev-next pointers
	SetPrevNext(he[0], he[1]);
	SetPrevNext(he[1], he[2]);
	SetPrevNext(he[2], he[0]);
	SetPrevNext(bhe[0], bhe[1]);
	SetPrevNext(bhe[1], bhe[2]);
	SetPrevNext(bhe[2], bhe[0]);

	// connect twin pointers
	SetTwin(he[0], bhe[0]);
	SetTwin(he[1], bhe[2]);
	SetTwin(he[2], bhe[1]);

	// connect start pointers for bhe
	bhe[0]->SetStart(v[1]);
	bhe[1]->SetStart(v[0]);
	bhe[2]->SetStart(v[2]);
	for (i = 0; i < 3; i++) he[i]->SetStart(v[i]);

	// connect start pointers
	// connect face-hedge pointers
	for (i = 0; i < 3; i++) {
		v[i]->SetHalfEdge(he[i]);
		v[i]->adjHEdges.push_back(he[i]);
		SetFace(f, he[i]);
	}
	v[0]->adjHEdges.push_back(bhe[1]);
	v[1]->adjHEdges.push_back(bhe[0]);
	v[2]->adjHEdges.push_back(bhe[2]);

	// mearge boundary if in need
	for (i = 0; i < 3; i++) {
		Vertex* start = bhe[i]->Start();
		Vertex* end = bhe[i]->End();
		for (size_t j = 0; j < end->adjHEdges.size(); j++) {
			HEdge* curr = end->adjHEdges[j];
			if (curr->IsBoundary() && curr->End() == start) {
				SetPrevNext(bhe[i]->Prev(), curr->Next());
				SetPrevNext(curr->Prev(), bhe[i]->Next());
				SetTwin(bhe[i]->Twin(), curr->Twin());
				bhe[i]->SetStart(NULL); // mark as unused
				curr->SetStart(NULL); // mark as unused
				break;
			}
		}
	}

	// finally add hedges and faces to list
	for (i = 0; i < 3; i++) heList.push_back(he[i]);
	for (i = 0; i < 3; i++) bheList.push_back(bhe[i]);
	fList.push_back(f);
}


void Mesh::Clear() {
	size_t i;
	for (i = 0; i < heList.size(); i++) delete heList[i];
	for (i = 0; i < bheList.size(); i++) delete bheList[i];
	for (i = 0; i < vList.size(); i++) delete vList[i];
	for (i = 0; i < fList.size(); i++) delete fList[i];
	heList.clear();
	bheList.clear();
	vList.clear();
	fList.clear();
}

Eigen::Vector3d Mesh::MinCoord() const {
	Eigen::Vector3d minCoord = Eigen::Vector3d::Zero();
	for (size_t i = 0; i < vList.size(); i++)
		minCoord = MinVector3d((vList[i])->Position(), minCoord);
	return minCoord;
}

Eigen::Vector3d Mesh::MaxCoord() const {
	Eigen::Vector3d maxCoord = Eigen::Vector3d::Zero();
	for (size_t i = 0; i < vList.size(); i++)
		maxCoord = MaxVector3d((vList[i])->Position(), maxCoord);
	return maxCoord;
}

void Mesh::DisplayMeshInfo() {
	/*************************/
	/* insert your code here */
	/*************************/
	int Num_vertices = vList.size();
	int Num_hedges = heList.size();
	int Num_bhedges = bheList.size();
	int Num_edges;
	int Num_fcaes = fList.size();

	int boundaries;
	int component;
	int genus;
	int Euler;

	Num_edges = (Num_hedges + Num_bhedges) / 2;
	boundaries = CountBoundaryLoops();
	component = CountConnectedComponents();
	Euler = Num_vertices - Num_edges + Num_fcaes;
	genus = (2 - Euler - boundaries) / 2;

	std::cout << "Number of Vertices: " << Num_vertices << std::endl;
	std::cout << "Number of Edges: " << Num_edges << std::endl;
	std::cout << "Number of Faces: " << Num_fcaes << std::endl;
	std::cout << "Number of Boundaries: " << boundaries << std::endl;
	std::cout << "Number of Genus: " << genus << std::endl;
	std::cout << "Number of Components: " << component << std::endl;
	std::cout << "Euler characteristic: " << Euler << std::endl;

}

// compute the normal of each vertex
void Mesh::ComputeVertexNormals() {

	const double PI = 3.14159265;
	Eigen::Vector3d t1, t2, nor, Ver1;

	int i, k, j;
	for (i = 0; i < vList.size(); i++) {

		//OneRingVertex ring(vList[i]);  // I didn't use the one ring function. it's a little bit confused especially for new comer of C++;

		k = vList[i]->Valence(); //get the degree of this vertex;
		HEdge* nextHedge;
		Vertex* nextVertex;

		t1 = Eigen::Vector3d(0, 0, 0); //define two vectors;
		t2 = Eigen::Vector3d(0, 0, 0);

		if (vList[i]->IsBoundary()) {

			nextHedge = vList[i]->HalfEdge()->Twin();  //get the first half edge and its twin, one ring vertex;
			nextVertex = nextHedge->Start();

			//make sure the starting half edge is boundary edge, so the next vertex is also on the boundary;
			//while (!(nextVertex->IsBoundary())){
			//nextHedge = nextHedge->Next()->Twin();
			//nextVertex = nextHedge->Start();
			//}

			Ver1 = nextVertex->Position(); //now, the starting vertex is denoted as Ver1;

			if (k == 2) {
				nextVertex = nextHedge->Next()->Twin()->Start();
				t1 = Ver1 - nextVertex->Position();
				t2 = Ver1 + nextVertex->Position() - vList[i]->Position();
			}
			else if (k == 3) {
				nextHedge = nextHedge->Next()->Twin();
				nextVertex = nextHedge->Start();
				t2 = nextVertex->Position() - vList[i]->Position();
				nextHedge = nextHedge->Next()->Twin();
				nextVertex = nextHedge->Start();
				t1 = Ver1 - nextVertex->Position();
			}
			else {
				for (j = 1; j < k - 1; j++) {
					nextHedge = nextHedge->Next()->Twin();
					nextVertex = nextHedge->Start();
					t2[0] += (2 * cos(PI / (k - 1)) - 2)* sin(j*PI / (k - 1))* nextVertex->Position()[0];
					t2[1] += (2 * cos(PI / (k - 1)) - 2)* sin(j*PI / (k - 1))* nextVertex->Position()[1];
					t2[2] += (2 * cos(PI / (k - 1)) - 2)* sin(j*PI / (k - 1))* nextVertex->Position()[2];

				}
				nextHedge = nextHedge->Next()->Twin();
				nextVertex = nextHedge->Start();
				t1 = Ver1 - nextVertex->Position();
				t2 = t2 + sin(PI / (k - 1))* t1;
			}

		}
		else {
			nextHedge = vList[i]->HalfEdge()->Twin();  //get the first half edge that points to this vertex;
			nextVertex = nextHedge->Start();

			for (j = 0; j < k; j++) {

				t1 += cos(2 * PI*j / k)*(nextVertex->Position() - vList[i]->Position());
				t2 += sin(2 * PI*j / k)*(nextVertex->Position() - vList[i]->Position());

				nextHedge = nextHedge->Next()->Twin(); //get next half edge that points to this vertex;
				nextVertex = nextHedge->Start();

			}

		}

		//set the normalied product of two vertors as vertex normal
		nor = -t1.cross(t2); // why it nagates itself??
		nor.normalize();
		vList[i]->SetNormal(nor);
	}


}


int Mesh::CountBoundaryLoops() {
	/*************************/
	/* insert your code here */
	/*************************/

	/*====== Programming Assignment 0 ======*/
	return 0;
}


int Mesh::CountConnectedComponents()
{
	/*************************/
	/* insert your code here */
	/*************************/

	/*====== Programming Assignment 0 ======*/
	return 0;
}


void Mesh::SetPrevNext(HEdge* e1, HEdge* e2) {
	e1->SetNext(e2);
	e2->SetPrev(e1);
}

void Mesh::SetTwin(HEdge* e1, HEdge* e2) {
	e1->SetTwin(e2);
	e2->SetTwin(e1);
}

void Mesh::SetFace(Face* f, HEdge* e) {
	f->SetHalfEdge(e);
	e->SetFace(f);
}

double Mesh::Cot(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const Eigen::Vector3d& p3) {
	Eigen::Vector3d v1 = p1 - p2;
	Eigen::Vector3d v2 = p3 - p2;

	double _dot_res = v1.normalized().dot(v2.normalized());
	if (_dot_res < -1.0) {
		_dot_res = -1.0;
	}
	else if (_dot_res > 1.0) {
		_dot_res = 1.0;
	}
	return 1.0 / std::tan(std::acos(_dot_res));
}

double Mesh::TriArea(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const Eigen::Vector3d& p3) {
	Eigen::Vector3d v1 = p2 - p1;
	Eigen::Vector3d v2 = p3 - p1;
	return v1.cross(v2).norm() / 2.0;
}


// Fill Hole
void Mesh::GeneratePatchMesh() {
	std::cout << "[step 1] Initial Patch Mesh Generation" << std::endl;

	bhefhList = bheList;
	for (int i = 0; i < bhefhList.size(); i++)
		bhefhList[i]->SetHole(true); // set bounadry to be in hole

	// average length l: s<>2*l
	double avg_l = 0;
	for (int i = 0; i < bhefhList.size(); i++) {
		Eigen::Vector3d len = bhefhList[i]->Start()->Position() - bhefhList[i]->End()->Position();
		avg_l += len.norm();
	}
	avg_l /= bhefhList.size();

	while (bhefhList.size() > 3) {
		// find smallest angle (cos)
		double max_cos = -1.0;
		Vertex * mp1, *mp2, *mp3;
		HEdge * he1, *he2;
		for (int i = 0; i < bhefhList.size(); i++) {
			Vertex * p1 = bhefhList[i]->Start();
			Vertex * p2 = bhefhList[i]->End();
			OneRingHEdge he(p1);
			HEdge * curr = he.NextHEdge();
			while (!curr->Twin()->IsHole())
				curr = he.NextHEdge();
			Vertex * p3 = curr->End();

			// min angle
			Eigen::Vector3d v1 = p2->Position() - p1->Position();
			Eigen::Vector3d v2 = p3->Position() - p1->Position();
			double cos_theta = v1.dot(v2) / (v1.norm()*v2.norm());
			if (cos_theta > max_cos) {
				max_cos = cos_theta;
				mp1 = p1;
				mp2 = p2;
				mp3 = p3;
				he1 = bhefhList[i];
				he2 = curr->Twin();
			}
		}

		// add HEdge, Vertex, Face
		double s = (mp2->Position() - mp3->Position()).norm();
		if (s < 2 * avg_l) {
			// add one HEdge
			Face * f = new Face();
			HEdge * he = new HEdge();
			HEdge * bhe = new HEdge();
			SetPrevNext(bhe, he1->Next());
			SetPrevNext(he2->Prev(), bhe);
			SetPrevNext(he1, he);
			SetPrevNext(he, he2);
			SetPrevNext(he2, he1);
			SetTwin(he, bhe);
			he->SetStart(mp2);
			bhe->SetStart(mp3);
			mp2->SetHalfEdge(he);
			mp3->SetHalfEdge(bhe);
			SetFace(f, he);
			SetFace(f, he1);
			SetFace(f, he2);
			ffhList.push_back(f);

			// update hole sign
			he1->SetHole(false);
			he2->SetHole(false);
			bhe->SetHole(true);
			bhefhList.push_back(bhe);
			heholeList.push_back(bhe);
		}
		else {
			// add two HEdge and one Vertex
			Face * f1 = new Face();
			Face * f2 = new Face();
			HEdge * h1 = new HEdge();
			HEdge * h2 = new HEdge();
			HEdge * h3 = new HEdge();
			HEdge * h4 = new HEdge();
			HEdge * b1 = new HEdge();
			HEdge * b2 = new HEdge();
			Vertex * p4 = new Vertex();
			p4->SetPosition((mp2->Position() + mp3->Position()) / 2);
			SetPrevNext(b1, he1->Next());
			SetPrevNext(he2->Prev(), b2);
			SetPrevNext(b2, b1);
			SetPrevNext(he1, h1);
			SetPrevNext(h1, h3);
			SetPrevNext(h3, he1);
			SetPrevNext(he2, h4);
			SetPrevNext(h4, h2);
			SetPrevNext(h2, he2);
			SetTwin(h3, h4);
			SetTwin(h1, b1);
			SetTwin(h2, b2);
			h1->SetStart(mp2);
			b1->SetStart(p4);
			h2->SetStart(p4);
			b2->SetStart(mp3);
			h3->SetStart(p4);
			h4->SetStart(mp1);
			mp2->SetHalfEdge(h1);
			mp3->SetHalfEdge(b2);
			p4->SetHalfEdge(h3);
			SetFace(f1, he1);
			SetFace(f1, h1);
			SetFace(f1, h3);
			SetFace(f2, he2);
			SetFace(f2, h4);
			SetFace(f2, h2);
			vfhList.push_back(p4);
			ffhList.push_back(f1);
			ffhList.push_back(f2);

			// update hole sign
			he1->SetHole(false);
			he2->SetHole(false);
			b1->SetHole(true);
			b2->SetHole(true);
			bhefhList.push_back(b1);
			bhefhList.push_back(b2);
			heholeList.push_back(b1);
			heholeList.push_back(b2);
			heholeList.push_back(h3);
		}

		// update bhefhList
		HEdgeList List;
		for (int i = 0; i < bhefhList.size(); i++) {
			if (bhefhList[i]->IsHole())
				List.push_back(bhefhList[i]);
		}
		bhefhList = List;
		//std::cout << "size: " << bhefhList.size() << std::endl;
	}
	// last 3 HEdges to form a face
	Face * f = new Face();
	HEdge *he[3];
	he[0] = bhefhList[0];
	if (bhefhList[0]->End() == bhefhList[1]->Start()) {
		he[1] = bhefhList[1];
		he[2] = bhefhList[2];
	}
	else if (bhefhList[0]->End() == bhefhList[2]->Start()) {
		he[1] = bhefhList[2];
		he[2] = bhefhList[1];
	}
	else {
		std::cout << "error" << std::endl;
		exit(0);
	}
	SetPrevNext(he[0], he[1]);
	SetPrevNext(he[1], he[2]);
	SetPrevNext(he[2], he[0]);
	for (int i = 0; i < 3; i++) {
		SetFace(f, he[i]);
		he[i]->SetHole(false);
	}
	ffhList.push_back(f);

	std::cout << "[step 1] Done" << std::endl;

	//OptimizeHole();
}

double avg_l = 0;
void Mesh::initFillHole() {
	bhefhList = bheList;
	for (int i = 0; i < bhefhList.size(); i++) {
		bhefhList[i]->SetHole(true); // HEdge to hole
	}

	// average length l: s<>2*l
	//double avg_l = 0;
	for (int i = 0; i < bhefhList.size(); i++) {
		Eigen::Vector3d len = bhefhList[i]->Start()->Position() - bhefhList[i]->End()->Position();
		avg_l += len.norm();
	}
	avg_l /= bhefhList.size();
}

void Mesh::FillHoleStep() {

	if (bhefhList.size() == 3) {
		// last 3 HEdges to form a face
		Face * f = new Face();
		HEdge *he[3];
		he[0] = bhefhList[0];
		if (bhefhList[0]->End() == bhefhList[1]->Start()) {
			he[1] = bhefhList[1];
			he[2] = bhefhList[2];
		}
		else if (bhefhList[0]->End() == bhefhList[2]->Start()) {
			he[1] = bhefhList[2];
			he[2] = bhefhList[1];
		}
		else {
			std::cout << "error" << std::endl;
		}
		SetPrevNext(he[0], he[1]);
		SetPrevNext(he[1], he[2]);
		SetPrevNext(he[2], he[0]);
		for (int i = 0; i < 3; i++) {
			SetFace(f, he[i]);
			he[i]->SetHole(false);
		}
		ffhList.push_back(f);
	}
	else if (bhefhList.size() > 3) {

		// find min angle (cos)
		double max_cos = -1.0;
		Vertex * mp1, *mp2, *mp3;
		HEdge * he1, *he2;
		for (int i = 0; i < bhefhList.size(); i++) {
			Vertex * p1 = bhefhList[i]->Start();
			Vertex * p2 = bhefhList[i]->End();
			OneRingHEdge he(p1);
			HEdge * curr = he.NextHEdge();
			while (!curr->Twin()->IsHole())
				curr = he.NextHEdge();
			Vertex * p3 = curr->End();

			// min angle
			Eigen::Vector3d v1 = p2->Position() - p1->Position();
			Eigen::Vector3d v2 = p3->Position() - p1->Position();
			double cos_theta = v1.dot(v2) / (v1.norm()*v2.norm());
			if (cos_theta > max_cos) {
				max_cos = cos_theta;
				mp1 = p1;
				mp2 = p2;
				mp3 = p3;
				he1 = bhefhList[i];
				he2 = curr->Twin();
			}
		}

		// add HEdge, Vertex, Face
		double s = (mp2->Position() - mp3->Position()).norm();
		if (s < 2 * avg_l) {
			// add one HEdge
			Face * f = new Face();
			HEdge * he = new HEdge();
			HEdge * bhe = new HEdge();
			SetPrevNext(bhe, he1->Next());
			SetPrevNext(he2->Prev(), bhe);
			SetPrevNext(he1, he);
			SetPrevNext(he, he2);
			SetPrevNext(he2, he1);
			SetTwin(he, bhe);
			he->SetStart(mp2);
			bhe->SetStart(mp3);
			mp2->SetHalfEdge(he);
			mp3->SetHalfEdge(bhe);
			SetFace(f, he);
			SetFace(f, he1);
			SetFace(f, he2);
			ffhList.push_back(f);

			// update hole sign
			he1->SetHole(false);
			he2->SetHole(false);
			bhe->SetHole(true);
			bhefhList.push_back(bhe);
			heholeList.push_back(bhe);
		}
		else {
			// add two HEdge and one Vertex
			Face * f1 = new Face();
			Face * f2 = new Face();
			HEdge * h1 = new HEdge();
			HEdge * h2 = new HEdge();
			HEdge * h3 = new HEdge();
			HEdge * h4 = new HEdge();
			HEdge * b1 = new HEdge();
			HEdge * b2 = new HEdge();
			Vertex * p4 = new Vertex();
			p4->SetPosition((mp2->Position() + mp3->Position()) / 2);
			SetPrevNext(b1, he1->Next());
			SetPrevNext(he2->Prev(), b2);
			SetPrevNext(b2, b1);
			SetPrevNext(he1, h1);
			SetPrevNext(h1, h3);
			SetPrevNext(h3, he1);
			SetPrevNext(he2, h4);
			SetPrevNext(h4, h2);
			SetPrevNext(h2, he2);
			SetTwin(h3, h4);
			SetTwin(h1, b1);
			SetTwin(h2, b2);
			h1->SetStart(mp2);
			b1->SetStart(p4);
			h2->SetStart(p4);
			b2->SetStart(mp3);
			h3->SetStart(p4);
			h4->SetStart(mp1);
			mp2->SetHalfEdge(h1);
			mp3->SetHalfEdge(b2);
			p4->SetHalfEdge(h3);
			SetFace(f1, he1);
			SetFace(f1, h1);
			SetFace(f1, h3);
			SetFace(f2, he2);
			SetFace(f2, h4);
			SetFace(f2, h2);
			vfhList.push_back(p4);
			ffhList.push_back(f1);
			ffhList.push_back(f2);

			// update hole sign
			he1->SetHole(false);
			he2->SetHole(false);
			b1->SetHole(true);
			b2->SetHole(true);
			bhefhList.push_back(b1);
			bhefhList.push_back(b2);
			heholeList.push_back(b1);
			heholeList.push_back(b2);
			heholeList.push_back(h3);
		}

		// update bhefhList
		HEdgeList List;
		for (int i = 0; i < bhefhList.size(); i++) {
			if (bhefhList[i]->IsHole())
				List.push_back(bhefhList[i]);
		}
		bhefhList = List;
		std::cout << "size: " << bhefhList.size() << std::endl;
	}
}


void Mesh::OptimizePatchMesh() {
	std::cout << "[step 2] Patch Mesh Optimization" << std::endl;

	VertexList vhbList; // vertex in hole and boundary
	for (int i = 0; i < vfhList.size(); i++)
		vhbList.push_back(vfhList[i]);
	for (int i = 0; i < bheList.size(); i++)
		vhbList.push_back(bheList[i]->Start());
	for (int i = 0; i < vhbList.size(); i++) {
		vhbList[i]->SetHole(true);
		vhbList[i]->SetHIndex(i);
	}

	// A = (L F)'
	Eigen::SparseMatrix<double, Eigen::ColMajor> A(vhbList.size() + bheList.size(), vhbList.size());
	std::vector<Eigen::Triplet<double>> triple;
	Eigen::VectorXd bx(vhbList.size() + bheList.size());
	Eigen::VectorXd by(vhbList.size() + bheList.size());
	Eigen::VectorXd bz(vhbList.size() + bheList.size());

	// construct L
	for (int i = 0; i < vfhList.size(); i++) {
		int d = vhbList[i]->Valence();
		OneRingHEdge ring(vhbList[i]);
		HEdge *he = NULL;
		while (he = ring.NextHEdge()) {
			triple.push_back(Eigen::Triplet<double>(i, he->End()->HIndex(), -1.0 / d));
		}
		triple.push_back(Eigen::Triplet<double>(i, i, 1));
		bx[i] = 0.0; by[i] = 0.0; bz[i] = 0.0;
	}
	for (int i = vfhList.size(); i < vhbList.size(); i++) {
		int d = 0;
		OneRingHEdge ring(vhbList[i]);
		HEdge *he = NULL;
		while (he = ring.NextHEdge()) {
			if (he->End()->IsHole())
				d++;
		}
		OneRingHEdge ring2(vhbList[i]);
		he = NULL;
		while (he = ring2.NextHEdge()) {
			if (he->End()->IsHole())
				triple.push_back(Eigen::Triplet<double>(i, he->End()->HIndex(), -1.0 / d));
		}
		triple.push_back(Eigen::Triplet<double>(i, i, 1));
		bx[i] = 0.0; by[i] = 0.0; bz[i] = 0.0;
	}
	// construct F
	for (int i = vhbList.size(); i < vhbList.size() + bheList.size(); i++) {
		triple.push_back(Eigen::Triplet<double>(i, i - bheList.size(), 1));
		bx[i] = vhbList[i - bheList.size()]->Position()(0);
		by[i] = vhbList[i - bheList.size()]->Position()(1);
		bz[i] = vhbList[i - bheList.size()]->Position()(2);
	}
	A.setFromTriplets(triple.begin(), triple.end());
	Eigen::SparseMatrix<double> M(vfhList.size(), vfhList.size());
	M = A.transpose()*A;
	Eigen::VectorXd ATbx = A.transpose()*bx;
	Eigen::VectorXd ATby = A.transpose()*by;
	Eigen::VectorXd ATbz = A.transpose()*bz;

	Eigen::SparseLU<Eigen::SparseMatrix<double, Eigen::ColMajor>> solver;
	solver.compute(M);

	Eigen::VectorXd vx = Eigen::VectorXd::Zero(vhbList.size());
	Eigen::VectorXd vy = Eigen::VectorXd::Zero(vhbList.size());
	Eigen::VectorXd vz = Eigen::VectorXd::Zero(vhbList.size());

	vx = solver.solve(ATbx);
	vy = solver.solve(ATby);
	vz = solver.solve(ATbz);

	for (int i = 0; i < vhbList.size(); i++) {
		Eigen::Vector3d newPosition = Eigen::Vector3d(vx[i], vy[i], vz[i]);
		vhbList[i]->SetPosition(newPosition);
	}
	std::cout << "[step 2] Done" << std::endl;

	ComputeBoundaryVertexNormals();
}


void Mesh::GenerateImplicitSurface() {
	/*
	define RBF mesh as:
	  f(x) = sum(lamda*RBF) + p0 + p1*x + p2*y + p3*z;
	construct matrix:
	  H*P = F
	solve it with LU
	*/
	std::cout << "[step 3] Implicit Surface Generation" << std::endl;

	int msize = 3 * bheList.size() + 4;
	// H
	Eigen::MatrixXd H(msize, msize);
	H = Eigen::MatrixXd::Zero(msize, msize);
	// P
	//Eigen::VectorXd P(msize);
	RBFSurfacePara = Eigen::VectorXd::Zero(msize);
	// F
	Eigen::VectorXd F(msize);

	//f(x) = 0, on the mesh
	for (int i = 0; i < bheList.size(); i++) {
		Eigen::Vector3d c = bheList[i]->Start()->Position();
		// sect 1
		for (int j = 0; j < bheList.size(); j++) {
			Eigen::Vector3d x = bheList[j]->Start()->Position();
			H(i, j) = RBF(x, c);
		}
		// sect 2
		for (int j = bheList.size(); j < 2 * bheList.size(); j++) {
			Eigen::Vector3d x = bheList[j - bheList.size()]->Start()->Position()
				+ bheList[j - bheList.size()]->Start()->Normal() / 100.0;
			H(i, j) = RBF(x, c);
		}
		// sect 3
		for (int j = 2 * bheList.size(); j < 3 * bheList.size(); j++) {
			Eigen::Vector3d x = bheList[j - 2 * bheList.size()]->Start()->Position()
				- bheList[j - 2 * bheList.size()]->Start()->Normal() / 100.0;
			H(i, j) = RBF(x, c);
		}
		H(i, 3 * bheList.size()) = 1;
		H(i, 3 * bheList.size() + 1) = c(0);
		H(i, 3 * bheList.size() + 2) = c(1);
		H(i, 3 * bheList.size() + 3) = c(2);
		H(3 * bheList.size(), i) = 1;
		H(3 * bheList.size() + 1, i) = c(0);
		H(3 * bheList.size() + 2, i) = c(1);
		H(3 * bheList.size() + 3, i) = c(2);
		F[i] = 0.0;
	}
	//f(x) = -1, outside
	for (int i = bheList.size(); i < 2 * bheList.size(); i++) {
		Eigen::Vector3d c = bheList[i - bheList.size()]->Start()->Position()
			+ bheList[i - bheList.size()]->Start()->Normal() / 100.0;
		// sect 4
		for (int j = 0; j < bheList.size(); j++) {
			Eigen::Vector3d x = bheList[j]->Start()->Position();
			H(i, j) = RBF(x, c);
		}
		// sect 5
		for (int j = bheList.size(); j < 2 * bheList.size(); j++) {
			Eigen::Vector3d x = bheList[j - bheList.size()]->Start()->Position()
				+ bheList[j - bheList.size()]->Start()->Normal() / 100.0;
			H(i, j) = RBF(x, c);
		}
		// sect 6
		for (int j = 2 * bheList.size(); j < 3 * bheList.size(); j++) {
			Eigen::Vector3d x = bheList[j - 2 * bheList.size()]->Start()->Position()
				- bheList[j - 2 * bheList.size()]->Start()->Normal() / 100.0;
			H(i, j) = RBF(x, c);
		}
		H(i, 3 * bheList.size()) = 1;
		H(i, 3 * bheList.size() + 1) = c(0);
		H(i, 3 * bheList.size() + 2) = c(1);
		H(i, 3 * bheList.size() + 3) = c(2);
		H(3 * bheList.size(), i) = 1;
		H(3 * bheList.size() + 1, i) = c(0);
		H(3 * bheList.size() + 2, i) = c(1);
		H(3 * bheList.size() + 3, i) = c(2);
		F[i] = -1.0;
	}
	//f(x) = 1, inside
	for (int i = 2 * bheList.size(); i < 3 * bheList.size(); i++) {
		Eigen::Vector3d c = bheList[i - 2 * bheList.size()]->Start()->Position()
			- bheList[i - 2 * bheList.size()]->Start()->Normal() / 100.0;
		// sect 7
		for (int j = 0; j < bheList.size(); j++) {
			Eigen::Vector3d x = bheList[j]->Start()->Position();
			H(i, j) = RBF(x, c);
		}
		// sect 8
		for (int j = bheList.size(); j < 2 * bheList.size(); j++) {
			Eigen::Vector3d x = bheList[j - bheList.size()]->Start()->Position()
				+ bheList[j - bheList.size()]->Start()->Normal() / 100.0;
		}
		// sect 9
		for (int j = 2 * bheList.size(); j < 3 * bheList.size(); j++) {
			Eigen::Vector3d x = bheList[j - 2 * bheList.size()]->Start()->Position()
				- bheList[j - 2 * bheList.size()]->Start()->Normal() / 100.0;
			H(i, j) = RBF(x, c);
		}
		H(i, 3 * bheList.size()) = 1;
		H(i, 3 * bheList.size() + 1) = c(0);
		H(i, 3 * bheList.size() + 2) = c(1);
		H(i, 3 * bheList.size() + 3) = c(2);
		H(3 * bheList.size(), i) = 1;
		H(3 * bheList.size() + 1, i) = c(0);
		H(3 * bheList.size() + 2, i) = c(1);
		H(3 * bheList.size() + 3, i) = c(2);
		F[i] = 1.0;
	}
	for (int i = msize - 4; i < msize; i++)
		F[i] = 0;

	// LU solver
	auto HSolver = H.lu();
	RBFSurfacePara = HSolver.solve(F);

	std::cout << "[step 3] Done" << std::endl;
}


double Mesh::RBF(Eigen::Vector3d x, Eigen::Vector3d c) {
	// c is control point, x is other point

	double l = (x - c).norm();
	if (abs(l) < 0.000001)
		return 0;
	//return pow(l, 3);
	return pow(l, 2)*log(l);
}


void Mesh::ComputeBoundaryVertexNormals() {

	for (int i = 0; i < bheList.size(); i++)
		bheList[i]->Start()->SetHole(false);

	for (int i = 0; i < bheList.size(); i++) {
		Eigen::Vector3d normal(0, 0, 0);
		Vertex *p1 = bheList[i]->Start();
		Vertex *p2, *p3;
		OneRingHEdge ring(p1);
		HEdge *he = NULL;
		while (he = ring.NextHEdge()) {
			if (!he->End()->IsHole()) {
				p2 = he->End();
				p3 = he->Twin()->Next()->End();
				if (p3->IsHole())
					continue;
				Eigen::Vector3d n = (p3->Position() - p1->Position()).cross(p2->Position() - p1->Position());
				normal += n / n.norm();
			}
		}
		normal.normalize();
		bheList[i]->Start()->SetNormal(normal);
	}

	for (int i = 0; i < bheList.size(); i++)
		bheList[i]->Start()->SetHole(true);

}


void Mesh::ProjectVertex() {
	std::cout << "[step 4] Vertex Projection" << std::endl;

	int count = 0;
	double error_v = -1.0;
	double th_v = 0.00000001; // threshold

	while(abs(error_v) > th_v) {
		// iteration
		for (int i = 0; i < vfhList.size(); i++) {
			Eigen::Vector3d xk = vfhList[i]->Position();
			double fval = SurfaceValue(xk);
			Eigen::Vector3d fgrad = SurfaceGrad(xk);
			xk += -fval / pow(fgrad.norm(), 2)*fgrad;
			vfhList[i]->SetPosition(xk);
		}
		// compute error
		error_v = 0;
		for (int i = 0; i < vfhList.size(); i++) {
			double v = SurfaceValue(vfhList[i]->Position());
			error_v += v;
		}
		error_v /= vfhList.size();
		count++;
		std::cout << "\titer count: " << count << "\terror:\t " << error_v << std::endl;
	} 

	std::cout << "[step 4] Done" << std::endl;
}


double Mesh::SurfaceValue(Eigen::Vector3d x) {

	double p0 = RBFSurfacePara[3 * bheList.size()];
	double p1 = RBFSurfacePara[3 * bheList.size() + 1];
	double p2 = RBFSurfacePara[3 * bheList.size() + 2];
	double p3 = RBFSurfacePara[3 * bheList.size() + 3];

	double f = p0 + p1 * x(0) + p2 * x(1) + p3 * x(2);

	for (int i = 0; i < bheList.size(); i++) {
		Eigen::Vector3d c = bheList[i]->Start()->Position();
		f += RBFSurfacePara[i] * RBF(x, c);
	}
	for (int i = bheList.size(); i < 2 * bheList.size(); i++) {
		Eigen::Vector3d c = bheList[i - bheList.size()]->Start()->Position()
			+ bheList[i - bheList.size()]->Start()->Normal() / 100.0;
		f += RBFSurfacePara[i] * RBF(x, c);
	}
	for (int i = 2 * bheList.size(); i < 3 * bheList.size(); i++) {
		Eigen::Vector3d c = bheList[i - 2 * bheList.size()]->Start()->Position()
			- bheList[i - 2 * bheList.size()]->Start()->Normal() / 100.0;
		f += RBFSurfacePara[i] * RBF(x, c);
	}

	return f;
}


Eigen::Vector3d Mesh::SurfaceGrad(Eigen::Vector3d x) {

	double p1 = RBFSurfacePara[3 * bheList.size() + 1];
	double p2 = RBFSurfacePara[3 * bheList.size() + 2];
	double p3 = RBFSurfacePara[3 * bheList.size() + 3];

	double fx = 0;
	double fy = 0;
	double fz = 0;

	for (int i = 0; i < bheList.size(); i++) {
		Eigen::Vector3d c = bheList[i]->Start()->Position();
		fx += RBFSurfacePara[i] * (x(0) - c(0))*(x - c).norm();
		fy += RBFSurfacePara[i] * (x(1) - c(1))*(x - c).norm();
		fz += RBFSurfacePara[i] * (x(2) - c(2))*(x - c).norm();
	}
	for (int i = bheList.size(); i < 2 * bheList.size(); i++) {
		Eigen::Vector3d c = bheList[i - bheList.size()]->Start()->Position()
			+ bheList[i - bheList.size()]->Start()->Normal() / 100.0;
		fx += RBFSurfacePara[i] * (x(0) - c(0))*(x - c).norm();
		fy += RBFSurfacePara[i] * (x(1) - c(1))*(x - c).norm();
		fz += RBFSurfacePara[i] * (x(2) - c(2))*(x - c).norm();
	}
	for (int i = 2 * bheList.size(); i < 3 * bheList.size(); i++) {
		Eigen::Vector3d c = bheList[i - 2 * bheList.size()]->Start()->Position()
			- bheList[i - 2 * bheList.size()]->Start()->Normal() / 100.0;
		fx += RBFSurfacePara[i] * (x(0) - c(0))*(x - c).norm();
		fy += RBFSurfacePara[i] * (x(1) - c(1))*(x - c).norm();
		fz += RBFSurfacePara[i] * (x(2) - c(2))*(x - c).norm();
	}

	fx = 3 * fx + p1;
	fy = 3 * fy + p2;
	fz = 3 * fz + p3;

	return Eigen::Vector3d(fx, fy, fz);
}


void Mesh::MergeMesh() {
	std::cout << "[final] Merge Mesh" << std::endl;

	for (int i = 0; i < vfhList.size(); i++) {
		vfhList[i]->SetIndex(vList.size() + i);
		vList.push_back(vfhList[i]);
	}
	for (int i = 0; i < ffhList.size(); i++) {
		fList.push_back(ffhList[i]);
	}
	ComputeVertexNormals();
	ComputeBoundaryVertexNormals();
	std::cout << "[final] Done" << std::endl;
}


