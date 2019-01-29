//%deps(assimp)

#include "geometry/import/read_collada.hh"

#include "out.hh"

#include <cassert>
#include <iostream>
#include <stack>

#include <assimp/postprocess.h>  // Post processing flags
#include <assimp/scene.h>        // Output data structure
#include <assimp/Importer.hpp>   // C++ importer interface

namespace geometry {
namespace import {
ColladaModel::ColladaModel(const std::string& path) {
  allocate(path);
}

namespace {
void explore(aiNode const* const node, int depth) {
  int n_children = node->mNumChildren;
  for (int k = 0; k < n_children; ++k) {
    for (int j = 0; j < depth; ++j) {
      std::cout << " ";
    }
    std::cout << node->mChildren[k]->mName.C_Str() << std::endl;
    explore(node->mChildren[k], depth + 1);
  }
}
Eigen::Matrix4d to_mat4(const aiMatrix4x4& mat) {
  const Eigen::Matrix4f outmat =
      (Eigen::Matrix4f() << mat[0][0], mat[0][1], mat[0][2], mat[0][3], mat[1][0],
       mat[1][1], mat[1][2], mat[1][3], mat[2][0], mat[2][1], mat[2][2], mat[2][3],
       mat[3][0], mat[3][1], mat[3][2], mat[3][3])
          .finished();
  return outmat.cast<double>();
}

Eigen::Vector3d to_vec3(const aiVector3D& mat) {
  const Eigen::Vector3d outmat = (Eigen::Vector3d() << mat[0], mat[1], mat[2]).finished();
  return outmat;
}

TriMesh assimp_mesh_to_trimesh(aiScene const* const scene, aiNode const* const node) {
  TriMesh tri_mesh;
  for (std::size_t mesh_ind = 0; mesh_ind < node->mNumMeshes; ++mesh_ind) {
    const aiMesh* mesh = scene->mMeshes[node->mMeshes[mesh_ind]];

    for (std::size_t face_ind = 0; face_ind < mesh->mNumFaces; ++face_ind) {
      const auto& face = mesh->mFaces[face_ind];

      assert(face.mNumIndices == 3);
      tri_mesh.triangles.emplace_back();
      for (int k = 0; k < 3; ++k) {
        const auto vertex = mesh->mVertices[face.mIndices[k]];
        tri_mesh.triangles.back().vertices[k] = to_vec3(vertex);
      }
    }
  }
  return tri_mesh;
}

void form(aiScene const* const scene,
          aiNode const* const root_node,
          Out<std::map<std::string, std::vector<ColladaModel::Edge>>> adjacency,
          Out<std::map<std::string, TriMesh>> meshes,
          Out<std::map<std::string, jcc::Vec4>> colors) {
  std::stack<aiNode const*> nodes;
  nodes.push(root_node);

  while (!nodes.empty()) {
    const aiNode* node = nodes.top();
    nodes.pop();
    const std::string this_node_name = node->mName.C_Str();

    if (meshes->count(this_node_name) > 0u) {
    }

    (*meshes)[this_node_name] = assimp_mesh_to_trimesh(scene, node);

    if (node->mNumMeshes) {
      const std::size_t material_ind = scene->mMeshes[node->mMeshes[0]]->mMaterialIndex;
      const aiMaterial* material = scene->mMaterials[material_ind];
      aiColor3D color_diff(0.f, 0.f, 0.f);
      material->Get(AI_MATKEY_COLOR_DIFFUSE, color_diff);

      (*colors)[this_node_name] =
          jcc::Vec4(color_diff[0], color_diff[1], color_diff[2], 0.7);
    }

    const int n_children = node->mNumChildren;
    for (int k = 0; k < n_children; ++k) {
      const aiNode* child = node->mChildren[k];

      ColladaModel::Edge edge;
      const Eigen::Matrix4d mat = to_mat4(child->mTransformation);

      edge.child_from_parent = SE3(mat).inverse();
      edge.child_name = child->mName.C_Str();

      nodes.push(child);
      if (adjacency->count(edge.child_name) == 0u) {
        // Don't allow repetitions -- this seems to be a bug in the OnShape Collada
        // exporter?
        (*adjacency)[this_node_name].push_back(edge);
      }
    }
  }
}

}  // namespace

void ColladaModel::allocate(const std::string& path) {
  Assimp::Importer importer;

  // This is leaked.
  const aiScene* scene =
      importer.ReadFile(path,
                        aiProcess_CalcTangentSpace | aiProcess_Triangulate |
                            aiProcess_JoinIdenticalVertices | aiProcess_SortByPType);
  assert(scene);

  root_ = scene->mRootNode->mName.C_Str();

  form(scene, scene->mRootNode, out(adjacency_), out(meshes_), out(colors_));
}

}  // namespace import
}  // namespace geometry
