#include "student_code.h"
#include "mutablePriorityQueue.h"

using namespace std;

namespace CGL
{
    
    float lerp(float t, double p0, double p1){
        return ((1-t)*p0)+(t*p1);
    }
    
  void BezierCurve::evaluateStep()
  {
    // TODO Part 1.
    // Perform one step of the Bezier curve's evaluation at t using de Casteljau's algorithm for subdivision.
    // Store all of the intermediate control points into the 2D vector evaluatedLevels.
      
      int level = evaluatedLevels.size() - 1;
      if (evaluatedLevels[level].size()<=1){
          return;
      }
      else{
          vector<Vector2D> newPoints = vector<Vector2D>();
          for (int i = 0; i<evaluatedLevels[level].size()-1; i++){
              double x0 = evaluatedLevels[level][i].x;
              double y0 = evaluatedLevels[level][i].y;
              double x1 = evaluatedLevels[level][i+1].x;
              double y1 = evaluatedLevels[level][i+1].y;
              float lerp0 = lerp(t,x0,x1);
              float lerp1 = lerp(t,y0,y1);
              newPoints.push_back(Vector2D(lerp0,lerp1));
          }
          evaluatedLevels.push_back(newPoints);
          return;
      }
      return;
  }
    
    


  Vector3D BezierPatch::evaluate(double u, double v) const
  {
    // TODO Part 2.
    // Evaluate the Bezier surface at parameters (u, v) through 2D de Casteljau subdivision.
    // (i.e. Unlike Part 1 where we performed one subdivision level per call to evaluateStep, this function
    // should apply de Casteljau's algorithm until it computes the final, evaluated point on the surface)
      vector<Vector3D> points = vector<Vector3D>();
      for (int i =0; i< controlPoints.size(); i++){
          points.push_back(evaluate1D(controlPoints[i], u));
      }
      return evaluate1D(points, v);
  }

  Vector3D BezierPatch::evaluate1D(std::vector<Vector3D> points, double t) const
  {
    // TODO Part 2.
    // Optional helper function that you might find useful to implement as an abstraction when implementing BezierPatch::evaluate.
    // Given an array of 4 points that lie on a single curve, evaluates the Bezier curve at parameter t using 1D de Casteljau subdivision.
      
      double x1 = lerp(t, points[0].x, points[1].x);
      double y1 = lerp(t, points[0].y, points[1].y);
      double z1 = lerp(t, points[0].z, points[1].z);
      Vector3D p1 = Vector3D(x1,y1,z1);
      double x2 = lerp(t, points[1].x, points[2].x);
      double y2 = lerp(t, points[1].y, points[2].y);
      double z2 = lerp(t, points[1].z, points[2].z);
      Vector3D p2 = Vector3D(x2,y2,z2);
      double x3 = lerp(t, points[2].x, points[3].x);
      double y3 = lerp(t, points[2].y, points[3].y);
      double z3 = lerp(t, points[2].z, points[3].z);
      Vector3D p3 = Vector3D(x3,y3,z3);
      
      double x11 = lerp(t, p1.x, p2.x);
      double y11 = lerp(t, p1.y, p2.y);
      double z11 = lerp(t, p1.z, p2.z);
      Vector3D p11 = Vector3D(x11, y11, z11);
      double x12 = lerp(t, p2.x, p3.x);
      double y12 = lerp(t, p2.y, p3.y);
      double z12 = lerp(t, p2.z, p3.z);
      Vector3D p12 = Vector3D(x12, y12, z12);
      
      double finalx = lerp(t, p11.x, p12.x);
      double finaly = lerp(t, p11.y, p12.y);
      double finalz = lerp(t, p11.z, p12.z);
      Vector3D finalVector = Vector3D(finalx, finaly, finalz);
    return finalVector;
 }



  Vector3D Vertex::normal( void ) const
  {
    // TODO Part 3.
    // TODO Returns an approximate unit normal at this vertex, computed by
    // TODO taking the area-weighted average of the normals of neighboring
    // TODO triangles, then normalizing.
      
      Vector3D n(0,0,0);
      HalfedgeCIter h = halfedge();
      h = h->twin();
      HalfedgeCIter h_orig = h;
      do{
          Vector3D v0 = h->vertex()->position;
          h = h->next();
          Vector3D v1 = h->vertex()->position;
          h = h->twin();
          Vector3D v2 = h->vertex()->position;
          
          Vector3D edge01 = v1 - v0;
          Vector3D edge12 = v2 - v1;
          n = n + cross(edge01, edge12);
      }
      while(h!= h_orig);
      
      return n.unit();
  }

  EdgeIter HalfedgeMesh::flipEdge( EdgeIter e0 )
  {
    // TODO Part 4.
    // TODO This method should flip the given edge and return an iterator to the flipped edge.
      if (e0->isBoundary()){
          return e0;
      }
      else{
          //next, twin, vertex, edge, face
          HalfedgeIter h0 = e0->halfedge();
          HalfedgeIter t0 = h0->twin();

          HalfedgeIter h1 = h0->next();
          HalfedgeIter t1 = h1->twin();

          HalfedgeIter h2 = h1->next();
          HalfedgeIter t2 = h2->twin();

          HalfedgeIter h3 = t0->next();
          HalfedgeIter t3 = h3->twin();

          HalfedgeIter h4 = h3->next();
          HalfedgeIter t4 = h4->twin();
          
          FaceIter f0 = h4->face();
          FaceIter f1 = h2->face();
          
          VertexIter v0 = h4->vertex();
          VertexIter v1 = h1->vertex();
          VertexIter v2 = h2->vertex();
          VertexIter v3 = h3->vertex();
          
          h0->setNeighbors(h4, t0, v2, e0, f1);
          h1->setNeighbors(h0, t1, v1, h1->edge(), f1);
          h2->setNeighbors(h3, t2, v2, h2->edge(), f0);
          h3->setNeighbors(t0, t3, v3, h3->edge(), f0);
          h4->setNeighbors(h1, t4, v0, h4->edge(), f1);
          t0->setNeighbors(h2, h0, v0, e0, f0);

          
          v0->halfedge() = h4;
          v1->halfedge() = h1;
          v2->halfedge()=h2;
          v3->halfedge()=h3;
          
          f0->halfedge()=h2;
          f1->halfedge()=h4;
          
          return e0;
      }
  }

  VertexIter HalfedgeMesh::splitEdge( EdgeIter e0 )
  {
    // TODO Part 5.
    // TODO This method should split the given edge and return an iterator to the newly inserted vertex.
    // TODO The halfedge of this vertex should point along the edge that was split, rather than the new edges.
      
      HalfedgeIter bc = e0->halfedge();
      HalfedgeIter cb = bc->twin();
      
      HalfedgeIter ca = bc->next();
      HalfedgeIter ac = ca->twin();
      
      HalfedgeIter ab = ca->next();
      HalfedgeIter ba = ab->twin();
      
      HalfedgeIter bd = cb->next();
      HalfedgeIter db = bd->twin();
      
      HalfedgeIter dc = bd->next();
      HalfedgeIter cd = dc->twin();
      
      FaceIter f0 = ca->face();
      FaceIter f1 = dc->face();
      
      VertexIter a = ab->vertex();
      VertexIter b = bd->vertex();
      VertexIter c = ca->vertex();
      VertexIter d = dc->vertex();
      
      Vector3D mposition = ((b->position) + (c->position))/2.0;
      
      VertexIter m = newVertex();
      m->position = mposition;
      
      HalfedgeIter cm = cb;
      HalfedgeIter mc = bc;
      HalfedgeIter bm = newHalfedge();
      HalfedgeIter mb = newHalfedge();
      HalfedgeIter am = newHalfedge();
      HalfedgeIter ma = newHalfedge();
      HalfedgeIter md = newHalfedge();
      HalfedgeIter dm = newHalfedge();
      
      FaceIter f2 = newFace();
      FaceIter f3 = newFace();
     
      EdgeIter Eam = newEdge();
      EdgeIter Emd = newEdge();
      EdgeIter Emb = newEdge();

      mc->setNeighbors(ca, cm, m, cm->edge(), f0);
      ca->setNeighbors(am, ac, c, ca->edge(), f0);
      am->setNeighbors(mc, ma, a, Eam, f0);
      
      cm->setNeighbors(md, mc, c, cm->edge(), f1);
      md->setNeighbors(dc, dm, m, Emd, f1);
      dc->setNeighbors(cm, cd, d, dc->edge(), f1);
      
      ma->setNeighbors(ab, am, m, Eam, f2);
      ab->setNeighbors(bm, ba, a, ab->edge(), f2);
      bm->setNeighbors(ma, mb, b, Emb, f2);
      
      mb->setNeighbors(bd, bm, m, Emb, f3);
      bd->setNeighbors(dm, db, b, bd->edge(), f3);
      dm->setNeighbors(mb, md, d, Emd, f3);
      
      f3->halfedge() = bd;
      f2->halfedge() = ab;
      f1->halfedge() = dc;
      f0->halfedge() = ca;
      
      Eam->halfedge() = am;
      Emd->halfedge() = md;
      Emb->halfedge() = mb;
      e0->halfedge()= mc;
      
      a->halfedge()=ab;
      b->halfedge() =bd;
      c->halfedge()=ca;
      d->halfedge()=dc;
      m->halfedge()=mc;
    return m;
  }



  void MeshResampler::upsample( HalfedgeMesh& mesh )
  {
    // TODO Part 6.
    // This routine should increase the number of triangles in the mesh using Loop subdivision.
    // Each vertex and edge of the original surface can be associated with a vertex in the new (subdivided) surface.
    // Therefore, our strategy for computing the subdivided vertex locations is to *first* compute the new positions
    // using the connectity of the original (coarse) mesh; navigating this mesh will be much easier than navigating
    // the new subdivided (fine) mesh, which has more elements to traverse. We will then assign vertex positions in
    // the new mesh based on the values we computed for the original mesh.


    // TODO Compute new positions for all the vertices in the input mesh, using the Loop subdivision rule,
    // TODO and store them in Vertex::newPosition. At this point, we also want to mark each vertex as being
    // TODO a vertex of the original mesh.
      
      for( VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++ ){
          HalfedgeCIter h = v->halfedge();
          h = h->twin();
          HalfedgeCIter h_orig = h;
          Vector3D neighbor_position_sum(0,0,0);
          do{
              neighbor_position_sum += h->vertex()->position;
              h = h->next()->twin();
          }
          while(h!= h_orig);
          float n = (float)v->degree();
          float u =  3.0 / (8.0*n);
          if (n==3.0){u=3.0/16.0;}
          Vector3D original_position = v->position;
          v->newPosition = ((1- (n*u))*original_position) + (u*neighbor_position_sum);
          v->isNew=false;
      }

    // TODO Next, compute the updated vertex positions associated with edges, and store it in Edge::newPosition.
      EdgeIter end;
      for( EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++ )
          {
              Vector3D A = e->halfedge()->vertex()->position;
              Vector3D B = e->halfedge()->twin()->vertex()->position;
              Vector3D C = e->halfedge()->next()->next()->vertex()->position;
              Vector3D D = e->halfedge()->twin()->next()->next()->vertex()->position;
              e->newPosition = ((3.0/8.0)*(A+B)) + ((1.0/8.0)*(C+D));
              e->isNew=false;
              end = e;
          }

    // TODO Next, we're going to split every edge in the mesh, in any order.  For future
    // TODO reference, we're also going to store some information about which subdivided
    // TODO edges come from splitting an edge in the original mesh, and which edges are new,
    // TODO by setting the flat Edge::isNew.  Note that in this loop, we only want to iterate
    // TODO over edges of the original mesh---otherwise, we'll end up splitting edges that we
    // TODO just split (and the loop will never end!)
      for (EdgeIter e = mesh.edgesBegin(); e != end; e++){
          VertexIter v = mesh.splitEdge(e);
          v->isNew=true;
          v->newPosition=e->newPosition;
          v->halfedge()->next()->next()->edge()->isNew=true;
          v->halfedge()->twin()->next()->edge()->isNew=true;
          v->halfedge()->edge()->isNew=false;
      }
      VertexIter v = mesh.splitEdge(end);
      v->isNew=true;
      v->newPosition=end->newPosition;
      v->halfedge()->next()->next()->edge()->isNew=true;
      v->halfedge()->twin()->next()->edge()->isNew=true;
      v->halfedge()->edge()->isNew=false;

    // TODO Now flip any new edge that connects an old and new vertex.
      for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++){
          if(e->isNew){
              VertexIter v1 = e->halfedge()->vertex();
              VertexIter v2 = e->halfedge()->twin()->vertex();
           if((v1->isNew==false && v2->isNew==true)|| (v1->isNew==true && v2->isNew==false)){
                  mesh.flipEdge(e);
              }
          }
      }

    // TODO Finally, copy the new vertex positions into final Vertex::position.
    for( VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++ ){
        v->position = v->newPosition;
    }
    return;
  }
}
