// Copyright dai zengtao. All Rights Reserved.


#include "GeometryScript/MeshSimplifyFunctionsPlus.h"

#include "ConstrainedDelaunay2.h"
#include "GroupTopology.h"
#include "MeshConstraints.h"
#include "MeshConstraintsUtil.h"
#include "MeshSimplification.h"
#include "ProjectionTargets.h"
#include "UDynamicMesh.h"
#include "DynamicMesh/MeshAttributeUtil.h"
#include "DynamicMesh/MeshNormals.h"
#include "Operations/PolygroupRemesh.h"

using namespace UE::Geometry;
#define LOCTEXT_NAMESPACE "UMeshSimplifyFunctionsPlus"

template <typename SimplificationType>
void ComputeSimplify(FDynamicMesh3* TargetMesh, const bool bReproject,
                     int OriginalTriCount, FDynamicMesh3& OriginalMesh, FDynamicMeshAABBTree3& OriginalMeshSpatial,
                     EEdgeRefineFlags MeshBoundaryConstraint,
                     EEdgeRefineFlags GroupBoundaryConstraint,
                     EEdgeRefineFlags MaterialBoundaryConstraint,
                     bool bPreserveSharpEdges, bool bAllowSeamCollapse,
                     const ERuntimeSimplifyTargetType TargetMode,
                     const float TargetPercentage, const int TargetCount, const float TargetEdgeLength,
                     const float AngleThreshold,
                     typename SimplificationType::ESimplificationCollapseModes CollapseMode,
                     bool bUseQuadricMemory,
                     float GeometricTolerance)
{
	SimplificationType Reducer(TargetMesh);

	Reducer.ProjectionMode = (bReproject)
		                         ? SimplificationType::ETargetProjectionMode::AfterRefinement
		                         : SimplificationType::ETargetProjectionMode::NoProjection;

	Reducer.DEBUG_CHECK_LEVEL = 0;
	//Reducer.ENABLE_PROFILING = true;

	Reducer.bAllowSeamCollapse = bAllowSeamCollapse;
	Reducer.bRetainQuadricMemory = bUseQuadricMemory;

	if (bAllowSeamCollapse)
	{
		Reducer.SetEdgeFlipTolerance(1.e-5);

		// eliminate any bowties that might have formed on UV seams.
		if (TargetMesh->Attributes())
		{
			TargetMesh->Attributes()->SplitAllBowties();
		}
	}

	FMeshConstraints constraints;
	FMeshConstraintsUtil::ConstrainAllBoundariesAndSeams(constraints, *TargetMesh,
	                                                     MeshBoundaryConstraint,
	                                                     GroupBoundaryConstraint,
	                                                     MaterialBoundaryConstraint,
	                                                     true, !bPreserveSharpEdges, bAllowSeamCollapse);
	Reducer.SetExternalConstraints(MoveTemp(constraints));

	// transfer constraint setting to the simplifier, these are used to update the constraints as edges collapse.	
	Reducer.MeshBoundaryConstraint = MeshBoundaryConstraint;
	Reducer.GroupBoundaryConstraint = GroupBoundaryConstraint;
	Reducer.MaterialBoundaryConstraint = MaterialBoundaryConstraint;

	if (TargetMode == ERuntimeSimplifyTargetType::MinimalPlanar)
	{
		Reducer.CollapseMode = SimplificationType::ESimplificationCollapseModes::AverageVertexPosition;
		GeometricTolerance = 0; // MinimalPlanar does not allow vertices to move off the input surface
	}
	else
	{
		Reducer.CollapseMode = CollapseMode;
	}

	// use projection target if we are reprojecting or doing geometric error checking
	FMeshProjectionTarget ProjTarget(&OriginalMesh, &OriginalMeshSpatial);
	if (bReproject || GeometricTolerance > 0)
	{
		Reducer.SetProjectionTarget(&ProjTarget);
	}

	// configure geometric error settings
	if (GeometricTolerance > 0)
	{
		Reducer.GeometricErrorConstraint =
			SimplificationType::EGeometricErrorCriteria::PredictedPointToProjectionTarget;
		Reducer.GeometricErrorTolerance = GeometricTolerance;
	}

	if (TargetMode == ERuntimeSimplifyTargetType::Percentage)
	{
		double Ratio = (double)TargetPercentage / 100.0;
		int UseTarget = FMath::Max(4, (int)(Ratio * (double)OriginalTriCount));
		Reducer.SimplifyToTriangleCount(UseTarget);
	}
	else if (TargetMode == ERuntimeSimplifyTargetType::TriangleCount)
	{
		Reducer.SimplifyToTriangleCount(TargetCount);
	}
	else if (TargetMode == ERuntimeSimplifyTargetType::VertexCount)
	{
		Reducer.SimplifyToVertexCount(TargetCount);
	}
	else if (TargetMode == ERuntimeSimplifyTargetType::EdgeLength)
	{
		Reducer.SimplifyToEdgeLength(TargetEdgeLength);
	}
	else if (TargetMode == ERuntimeSimplifyTargetType::MinimalPlanar)
	{
		Reducer.SimplifyToMinimalPlanar(AngleThreshold);
	}
}

UDynamicMesh* UMeshSimplifyFunctionsPlus::ApplySimplifyToTargetPercentage(
	UDynamicMesh* TargetMesh, FGeometryScriptPlusSimplifyMeshOptions Options, int32 TargetPercentage,
	UGeometryScriptDebug* Debug)
{
	if (TargetMesh == nullptr)
	{
		AppendError(Debug, EGeometryScriptErrorType::InvalidInputs,
		            LOCTEXT("ApplySimplifyToTargetPercentage_InvalidInput",
		                    "ApplySimplifyToTargetPercentage: TargetMesh is Null"));
		return TargetMesh;
	}

	CalculateResult(TargetMesh, Options, ERuntimeSimplifyTargetType::Percentage, TargetPercentage);

	return TargetMesh;
}

UDynamicMesh* UMeshSimplifyFunctionsPlus::CalculateResult(
	UDynamicMesh* TargetMesh, FGeometryScriptPlusSimplifyMeshOptions Options,
	const ERuntimeSimplifyTargetType TargetMode,
	const float TargetPercentage, const int TargetCount, const float TargetEdgeLength,
	const float PolyEdgeAngleTolerance)
{
	/**
	 * all default
	 * @link EMeshBoundaryConstraint @endlink
	 */
	EEdgeRefineFlags MeshBoundaryConstraint = EEdgeRefineFlags::NoFlip;
	EEdgeRefineFlags GroupBoundaryConstraint = EEdgeRefineFlags::NoConstraint;
	EEdgeRefineFlags MaterialBoundaryConstraint = EEdgeRefineFlags::NoConstraint;
	/** If true, sharp edges are preserved  */
	// bool bPreserveSharpEdges = false;
	bool bPreserveSharpEdges = true;
	bool bAllowSeamCollapse = bPreserveSharpEdges;
	/** Prevent normal flips */
	bool bPreventNormalFlips = true;
	/** Prevent introduction of tiny triangles or slivers */
	bool bPreventTinyTriangles = false;
	/** Angle threshold in degrees used for testing if two triangles should be considered coplanar, or two lines collinear */
	float MinimalAngleThreshold = 0.01;
	float MinimalPlanarAngleThresh = MinimalAngleThreshold;
	bool bResultMustHaveAttributesEnabled = true;

	TUniquePtr<FDynamicMesh3> ResultMesh = MakeUnique<FDynamicMesh3>();
	// Need access to the source mesh:
	FDynamicMesh3* TargetMesh3 = ResultMesh.Get();
	FDynamicMesh3* OriginalMesh = TargetMesh->GetMeshPtr();
	TSharedPtr<FDynamicMeshAABBTree3, ESPMode::ThreadSafe> OriginalMeshSpatial = MakeShared<
		FDynamicMeshAABBTree3, ESPMode::ThreadSafe>(OriginalMesh, true);
	int OriginalTriCount = OriginalMesh->TriangleCount();
	double UseGeometricTolerance = Options.bGeometricConstraint ? Options.GeometricTolerance : 0.0;
	if (Options.SimplifyType == ERuntimeSimplifyType::QEM)
	{
		bool bUseQuadricMemory = true;
		ResultMesh->Copy(*OriginalMesh, true, true, true, !Options.bDiscardAttributes);
		ComputeSimplify<FQEMSimplification>(TargetMesh3, Options.bReproject, OriginalTriCount, *OriginalMesh,
		                                    *OriginalMeshSpatial,
		                                    MeshBoundaryConstraint, GroupBoundaryConstraint, MaterialBoundaryConstraint,
		                                    bPreserveSharpEdges, bAllowSeamCollapse,
		                                    TargetMode, TargetPercentage, TargetCount, TargetEdgeLength,
		                                    MinimalPlanarAngleThresh,
		                                    FQEMSimplification::ESimplificationCollapseModes::MinimalQuadricPositionError,
		                                    bUseQuadricMemory,
		                                    UseGeometricTolerance);
		int ResultTriCount = OriginalMesh->TriangleCount();
	}
	else if (Options.SimplifyType == ERuntimeSimplifyType::Attribute)
	{
		bool bUseQuadricMemory = false;
		ResultMesh->Copy(*OriginalMesh, true, true, true, !Options.bDiscardAttributes);
		if (!ResultMesh->HasAttributes() && !ResultMesh->HasVertexNormals())
		{
			FMeshNormals::QuickComputeVertexNormals(*ResultMesh, false);
		}
		ComputeSimplify<FAttrMeshSimplification>(TargetMesh3, Options.bReproject, OriginalTriCount, *OriginalMesh,
		                                         *OriginalMeshSpatial,
		                                         MeshBoundaryConstraint,
		                                         GroupBoundaryConstraint,
		                                         MaterialBoundaryConstraint,
		                                         bPreserveSharpEdges, bAllowSeamCollapse,
		                                         TargetMode, TargetPercentage, TargetCount, TargetEdgeLength,
		                                         MinimalPlanarAngleThresh,
		                                         FAttrMeshSimplification::ESimplificationCollapseModes::MinimalQuadricPositionError,
		                                         bUseQuadricMemory,
		                                         UseGeometricTolerance);
	}
	else if (Options.SimplifyType == ERuntimeSimplifyType::MinimalPlanar)
	{
		bool bUseQuadricMemory = false;
		ResultMesh->Copy(*OriginalMesh, true, true, true, !Options.bDiscardAttributes);
		if (!ResultMesh->HasAttributes() && !ResultMesh->HasVertexNormals())
		{
			FMeshNormals::QuickComputeVertexNormals(*ResultMesh, false);
		}
		ComputeSimplify<FQEMSimplification>(TargetMesh3, Options.bReproject, OriginalTriCount, *OriginalMesh,
		                                    *OriginalMeshSpatial,
		                                    MeshBoundaryConstraint,
		                                    GroupBoundaryConstraint,
		                                    MaterialBoundaryConstraint,
		                                    bPreserveSharpEdges, bAllowSeamCollapse,
		                                    ERuntimeSimplifyTargetType::MinimalPlanar, TargetPercentage, TargetCount,
		                                    TargetEdgeLength, MinimalPlanarAngleThresh,
		                                    FQEMSimplification::ESimplificationCollapseModes::MinimalQuadricPositionError,
		                                    bUseQuadricMemory,
		                                    UseGeometricTolerance);
	}
	else if (Options.SimplifyType == ERuntimeSimplifyType::MinimalExistingVertex)
	{
		bool bUseQuadricMemory = true;
		ResultMesh->Copy(*OriginalMesh, true, true, true, !Options.bDiscardAttributes);
		ComputeSimplify<FQEMSimplification>(TargetMesh3, Options.bReproject, OriginalTriCount, *OriginalMesh,
		                                    *OriginalMeshSpatial,
		                                    MeshBoundaryConstraint,
		                                    GroupBoundaryConstraint,
		                                    MaterialBoundaryConstraint,
		                                    bPreserveSharpEdges, bAllowSeamCollapse,
		                                    TargetMode, TargetPercentage, TargetCount, TargetEdgeLength,
		                                    MinimalPlanarAngleThresh,
		                                    FQEMSimplification::ESimplificationCollapseModes::MinimalExistingVertexError,
		                                    bUseQuadricMemory,
		                                    UseGeometricTolerance);
	}
	else if (Options.SimplifyType == ERuntimeSimplifyType::MinimalPolygroup)
	{
		ResultMesh->Copy(*OriginalMesh, true, true, true, !Options.bDiscardAttributes);
		FGroupTopology Topology(ResultMesh.Get(), true);
		FPolygroupRemesh Remesh(ResultMesh.Get(), &Topology, ConstrainedDelaunayTriangulate<double>);
		Remesh.SimplificationAngleTolerance = PolyEdgeAngleTolerance;
		Remesh.Compute();
	}

	if (!ResultMesh->HasAttributes())
	{
		FMeshNormals::QuickComputeVertexNormals(*ResultMesh);
	}

	if (!TargetMesh3->HasAttributes() && bResultMustHaveAttributesEnabled)
	{
		TargetMesh3->EnableAttributes();
		if (TargetMesh3->HasVertexUVs())
		{
			CopyVertexUVsToOverlay(*TargetMesh3, *TargetMesh3->Attributes()->PrimaryUV());
		}
		if (TargetMesh3->HasVertexNormals())
		{
			CopyVertexNormalsToOverlay(*TargetMesh3, *TargetMesh3->Attributes()->PrimaryNormals());
		}
	}

	TargetMesh->SetMesh(*ResultMesh);
	return TargetMesh;
}

#undef LOCTEXT_NAMESPACE
