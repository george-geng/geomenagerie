/*
 *
 *
 * Implemented by George Geng.
 *
 */

#include "student_code.h"
#include "mutablePriorityQueue.h"


namespace CGL {

    void BezierPatch::preprocess() {

        //called in bezierPatch.cpp where control points are already loaded, u can just call control points
        //turn control points into 3 x y z matrices
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                xMatrix(i,j) = controlPoints[i][j].x;
                yMatrix(i,j) = controlPoints[i][j].y;
                zMatrix(i,j) = controlPoints[i][j].z;
            }
        }
        //matrices used to compute the bezier matrices
        Matrix4x4 M;
        M(0,0) = 1; M(0,1) = 0; M(0,2) = 0; M(0,3) = 0;
        M(1,0) = -3; M(1,1) = 3; M(1,2) = 0; M(1,3) = 0;
        M(2,0) = 3; M(2,1) = -6; M(2,2) = 3; M(2,3) = 0;
        M(3,0) = -1; M(3,1) = 3; M(3,2) = -3; M(3,3) = 1;

        xBezierMatrix = M* xMatrix* M.T();
        yBezierMatrix = M* yMatrix* M.T();
        zBezierMatrix = M* zMatrix* M.T();
    }

    Vector3D BezierPatch::evaluate(double u, double v) const {
        // TODO Part 1.
        // Vector4D uVector = Vector4D(u*u*u, u*u, u, 1);
        // Vector4D vVector = Vector4D(v*v*v, v*v, v, 1); 
        Vector4D uVector = Vector4D(1, u, u*u, u*u*u);
        Vector4D vVector = Vector4D(1, v, v*v, v*v*v);
        double x = dot(uVector, (xBezierMatrix*vVector));
        double y = dot(uVector, (yBezierMatrix*vVector));
        double z = dot(uVector, (zBezierMatrix*vVector));
        Vector3D toReturn = Vector3D(x,y, z);
        return toReturn;
    }

    void BezierPatch::add2mesh(Polymesh* mesh) const {
        // TODO Part 1.
        Vector3D v1;
        Vector3D v2;
        Vector3D v3;
        Vector3D v4;

        //iterate through the 9x9 grid of Vectord3Ds
        for (double v = 0; v < 1; v+=0.125) {
            for (double u = 0; u < 1; u += 0.125) {
                v1 = evaluate(u, v);
                v2 = evaluate(u + 0.125, v);
                v3 = evaluate(u, v + 0.125);
                v4 = evaluate(u + 0.125, v + 0.125);
                addTriangle(mesh, v4, v2, v1);
                addTriangle(mesh, v3, v4, v1);
            }
        }

    }

    Vector3D Vertex::normal(void) const {
    // TODO Part 2.
        //return Vector3D();
        Vector3D vertNorm = Vector3D(0,0,0);
        HalfedgeCIter h = halfedge();
        h = h->twin();
        HalfedgeCIter hTwin = h;
        do{
            if(!h->face()->isBoundary()) {
                vertNorm += h->face()->normal();
            }
            h = h->next()->twin();
        } while (h != hTwin);
        return vertNorm.unit();
    }

    EdgeIter HalfedgeMesh::flipEdge(EdgeIter e0) {
        // TODO Part 3.
        //return if boundary loops
        // TODO Part 3.
        //return if boundary loops
        HalfedgeIter h0 = e0->halfedge();
        if(e0->halfedge()->isBoundary()
            || e0->halfedge()->twin()->isBoundary()) {
            return EdgeIter();
        }
        //get half edges
        HalfedgeIter h1 = h0->next();
        HalfedgeIter h2 = h1->next();
        HalfedgeIter h3 = h0->twin();
        HalfedgeIter h4 = h3->next();
        HalfedgeIter h5 = h4->next();
        HalfedgeIter h6 = h1->twin();
        HalfedgeIter h7 = h2->twin();
        HalfedgeIter h8 = h4->twin();
        HalfedgeIter h9 = h5->twin();
        //get vertices
        VertexIter v0 = h0->vertex();
        VertexIter v1 = h3->vertex();
        VertexIter v2 = h2->vertex();
        VertexIter v3 = h5->vertex();
        //get faces
        FaceIter f0 = h0->face();
        FaceIter f1 = h3->face();
        //get edges
        EdgeIter e1 = h1->edge();
        EdgeIter e2 = h2->edge();
        EdgeIter e3 = h4->edge();
        EdgeIter e4 = h5->edge();

        h0->setNeighbors(h1, h3, v3, e0, f0);
        h1->setNeighbors(h2, h7, v2, e2, f0);
        h2->setNeighbors(h0, h8, v0, e3, f0);
        h3->setNeighbors(h4, h0, v2, e0, f1);
        h4->setNeighbors(h5, h9, v3, e4, f1);
        h5->setNeighbors(h3, h6, v1, e1, f1);
        //Set outside edges (differently!)
        //set h6
        h6->next() = h6->next();
        h6->twin() = h5;
        h6->vertex() = v2;
        h6->edge() = e1;
        h6->face() = h6->face();
        //set h7
        h7->next() = h7->next();
        h7->twin() = h1;
        h7->vertex() = v0;
        h7->edge() = e2;
        h7->face() = h7->face();
        //set h8
        h8->next() = h8->next();
        h8->twin() = h2;
        h8->vertex() = v3;
        h8->edge() = e3;
        h8->face() = h8->face();
        //set h9
        h9->next() = h9->next();
        h9->twin() = h4;
        h9->vertex() = v1;
        h9->edge() = e4;
        h9->face() = h9->face();
        //set vertices
        v0->halfedge() = h2; 
        v1->halfedge() = h5;
        v2->halfedge() = h3;
        v3->halfedge()= h0;
        //set edges
        e0->halfedge() = h0;
        e1->halfedge() = h5;
        e2->halfedge() = h1;
        e3->halfedge() = h2;
        e4->halfedge() = h4;
        //set faces
        f0->halfedge() = h0; 
        f1->halfedge() = h3;
        h6->face()->halfedge() = h6;
        h7->face()->halfedge() = h7;
        h8->face()->halfedge() = h8;
        h9->face()->halfedge() = h9;


        return e0;
   
    }

    VertexIter HalfedgeMesh::splitEdge(EdgeIter e0) {
        // TODO Part 4.
        //see if boundary
        if(e0->halfedge()->isBoundary()
            || e0->halfedge()->twin()->isBoundary()) {
            return VertexIter();
        }
        //First, gather all mesh elements as before
        //get half edges
        HalfedgeIter h0 = e0->halfedge();
        HalfedgeIter h1 = h0->next();
        HalfedgeIter h2 = h1->next();
        HalfedgeIter h3 = h0->twin();
        HalfedgeIter h4 = h3->next();
        HalfedgeIter h5 = h4->next();
        HalfedgeIter h6 = h1->twin();
        HalfedgeIter h7 = h2->twin();
        HalfedgeIter h8 = h4->twin();
        HalfedgeIter h9 = h5->twin();
        //get vertices
        VertexIter v0 = h0->vertex();
        VertexIter v1 = h3->vertex();
        VertexIter v2 = h2->vertex();
        VertexIter v3 = h5->vertex();
        //get faces
        FaceIter f0 = h0->face();
        FaceIter f1 = h3->face();
        //get edges
        EdgeIter e1 = h1->edge();
        EdgeIter e2 = h2->edge();
        EdgeIter e3 = h4->edge();
        EdgeIter e4 = h5->edge();

        //Create new stuff, allocation
        //create the new vertex
        VertexIter v4 = newVertex();
        //create new edges
        EdgeIter e5 = newEdge();
        EdgeIter e6 = newEdge();
        EdgeIter e7 = newEdge();
        //create new half edges
        HalfedgeIter h10 = newHalfedge();
        HalfedgeIter h11 = newHalfedge(); 
        HalfedgeIter h12 = newHalfedge();
        HalfedgeIter h13 = newHalfedge();
        HalfedgeIter h14 = newHalfedge();
        HalfedgeIter h15 = newHalfedge();
        //create new faces
        FaceIter f2 = newFace();
        FaceIter f3 = newFace();
        //now set everything
        
        //set up new pointers for halfedge
        h0->setNeighbors(h1, h3, v4, e0, f0);
        h1->setNeighbors(h2, h6, v1, e1, f0);
        h2->setNeighbors(h0, h11, v2, e5, f0);
        //  printf("f0 all good\n");
        h3->setNeighbors(h4, h0, v1, e0, f1);
        h4->setNeighbors(h5, h15, v4, e7, f1);
        h5->setNeighbors(h3, h9, v3, e4, f1);
        //printf("f1 all good\n");
        h6->setNeighbors(h6->next(), h1, v2, e1, h6->face());
       // printf("f6 all good\n");
        h7->setNeighbors(h7->next(), h12, v0, e2, h7->face());
       // printf("f7 all good\n");
        h8->setNeighbors(h8->next(), h14, v3, e3, h8->face());
      //  printf("f8 all good\n");
        h9->setNeighbors(h9->next(), h5, v1, e4, h9->face()); 
       // printf("f9 all good\n"); 
        h10->setNeighbors(h11, h13, v0, e6, f2);
        h11->setNeighbors(h12, h2, v4, e5, f2);
        h12->setNeighbors(h10, h7, v2, e2, f2);
      //  printf("f2 all good\n");
        h13->setNeighbors(h14, h10, v4, e6, f3);
        h14->setNeighbors(h15, h8, v0, e3, f3);
        h15->setNeighbors(h13, h4, v3, e7, f3);
       // printf("f3 all good\n");
        //reset vertices
        v0->halfedge() = h10;
        v1->halfedge() = h3;
        v2->halfedge() = h2;
        v3->halfedge() = h5;
        v4->position = 0.5*((v1->position) + (v0->position));
        v4->halfedge() = h0;

        //reset edges
        e0->halfedge() = h0;
        e1->halfedge() = h1;
        e2->halfedge() = h12;
        e3->halfedge() = h14;
        e4->halfedge() = h5;
        e5->halfedge() = h2;
        e6->halfedge() = h13;
        e7->halfedge() = h4;
        //reset faces
        f0->halfedge() = h0;
        f1->halfedge() = h3;
        f2->halfedge() = h10;
        f3->halfedge() = h13;
        h6->face()->halfedge() = h6;
        h7->face()->halfedge() = h7;
        h8->face()->halfedge() = h8;
        h9->face()->halfedge() = h9;

        return v4;
    }

    //helper functino for part 5;
    bool oldNew (EdgeIter e) {
        VertexIter v0 = e->halfedge()->vertex();
        VertexIter v1 = e->halfedge()->twin()->vertex();
        return (((v0->isNew) && !(v1->isNew)) || (!(v0->isNew) && (v1->isNew)));
    }

    void MeshResampler::upsample(HalfedgeMesh& mesh)

    // this routine increases the number of triangles in the mesh using Loop subdivision.
    {
        //Set all existing vertices isNew to false and compute new position
         for(VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
            v->isNew = false;
            //also compute and set new position of original vertices
            float n = (double)(v->degree());
            Vector3D positionSum;
            HalfedgeIter h = v->halfedge();
            h = h->twin();
            HalfedgeIter hTwin = h;
            do{
                //if(!h->face()->isBoundary()) {
                    positionSum += h->vertex()->position;
                 //}
                h = h->next()->twin();
            } while (h != hTwin);
           
            double u;
            if (n == 3.0) {
                u = (3.0/16.0);
            } else {
                u = 3/(8*(double)n);
            }
            v->newPosition = (1 - (double)n*u) * (v->position) + u * positionSum;
        }
        // Compute new positions for vertices that will be inserted at edge midpoints in edge->newPosition.
         for (EdgeIter e = mesh.edgesBegin(); e!=mesh.edgesEnd(); e++) {
            if(e->halfedge()->isBoundary()
                || e->halfedge()->twin()->isBoundary()) {
                continue;
            }
            e->isNew = false;
            Vector3D aPos = e->halfedge()->vertex()->position;
            Vector3D bPos = e->halfedge()->twin()->vertex()->position;
            Vector3D cPos = e->halfedge()->next()->next()->vertex()->position;
            Vector3D dPos = e->halfedge()->twin()->next()->next()->vertex()->position;
            e->newPosition = 0.375*(aPos+bPos) + 0.125*(cPos+dPos);
         }
         //split edges that are NOT new or do NOT connect and old and new vertex
        for (EdgeIter e = mesh.edgesBegin(); e!= mesh.edgesEnd(); e++) {
            if(e->halfedge()->isBoundary()
                || e->halfedge()->twin()->isBoundary()) {
                continue;
            }
            if (!e->isNew && !oldNew(e)) {
                VertexIter v = mesh.splitEdge(e);
                v->isNew = true;
                v->newPosition = e->newPosition;
                v->halfedge()->edge()->isNew = false;
                v->halfedge()->twin()->next()->edge()->isNew = true;
                v->halfedge()->twin()->next()->twin()->next()->edge()->isNew = false;
                v->halfedge()->twin()->next()->twin()->next()->twin()->next()->edge()->isNew = true;     
            }
        }
        // flip NEW edges that connect old and new vertx
        for (EdgeIter e = mesh.edgesBegin(); e!= mesh.edgesEnd(); e++) {
            if(e->halfedge()->isBoundary()
                || e->halfedge()->twin()->isBoundary()) {
                continue;
            }
            if (e->isNew && oldNew(e)) {
                mesh.flipEdge(e);
            }
        }
        // update vertex positions, be careful to skip those at the boundaries tho
        for (VertexIter v = mesh.verticesBegin(); v!= mesh.verticesEnd(); v++) {
            if(v->isBoundary()) {
                continue;
            }
            v->position = v->newPosition;
        }
    }

   

   //also complete code "Shader/frag" file.

}
