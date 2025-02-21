// Copyright dai zengtao. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "GeometryScript/GeometryScriptTypes.h"
#include "MeshSimplifyFunctionsPlus.generated.h"

class UDynamicMesh;
class UGeometryScriptDebug;

/**
 * @link ESimplifyTargetType  @endlink
 */
UENUM(BlueprintType)
enum class ERuntimeSimplifyTargetType : uint8
{
	/** Percentage of input triangles */
	Percentage = 0 UMETA(DisplayName = "Percentage"),

	/** Target triangle count */
	TriangleCount = 1 UMETA(DisplayName = "Triangle Count"),

	/** Target vertex count */
	VertexCount = 2 UMETA(DisplayName = "Vertex Count"),

	/** Target edge length */
	EdgeLength = 3 UMETA(DisplayName = "Edge Length"),

	/** Apply all allowable edge collapses that do not change the shape */
	MinimalPlanar = 4 UMETA(Hidden)
};

/**
 * @link ESimplifyType @endlink  but runtime, without UEStandard
 */
UENUM(BlueprintType)
enum class ERuntimeSimplifyType : uint8
{
	/** Fastest. Standard quadric error metric.*/
	QEM = 0 UMETA(DisplayName = "QEM"),

	/** Potentially higher quality. Takes the normal into account. */
	Attribute = 1 UMETA(DisplayName = "Normal Aware"),

	/** Edge collapse to existing vertices only.  Quality may suffer.*/
	MinimalExistingVertex = 3 UMETA(DisplayName = "Existing Positions"),

	/** Collapse any spurious edges but do not change the 3D shape. */
	MinimalPlanar = 4 UMETA(DisplayName = "Minimal Shape-Preserving"),

	/** Only preserve polygroup boundaries; ignore all other shape features */
	MinimalPolygroup = 5 UMETA(DisplayName = "Minimal Polygroup-Preserving"),
};

/**
 * @link USimplifyMeshToolProperties  @endlink
 * @link FSimplifyMeshOp  @endlink
 */
USTRUCT(Blueprintable)
struct GEOMETRYSCRIPTINGPLUS_API FGeometryScriptPlusSimplifyMeshOptions
{
	GENERATED_BODY()

public:
	/** Simplified approach, where UEStandard cannot be used at runtime*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Options)
	ERuntimeSimplifyType SimplifyType = ERuntimeSimplifyType::QEM;

	/** If true, UVs and Normals are discarded  */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Options)
	bool bDiscardAttributes = false;

	/** If true, then simplification will consider geometric deviation with the input mesh  */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Options,
		meta = (EditCondition = "SimplifierType != ERuntimeSimplifyType::MinimalPolygroup"))
	bool bGeometricConstraint = false;

	/** Geometric deviation tolerance used when bGeometricConstraint is enabled, to limit the geometric deviation between the simplified and original meshes */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Options,
		meta = (UIMin = "0.0", UIMax = "10.0", ClampMin = "0.0", ClampMax = "10000000.0",
			EditCondition ="bGeometricConstraint && SimplifierType != ERuntimeSimplifyType::MinimalPolygroup"))
	float GeometricTolerance = 0.0f;

	/** Enable projection back to input mesh */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Options)
	bool bReproject = false;
};

/**
 * 
 */
UCLASS()
class GEOMETRYSCRIPTINGPLUS_API UMeshSimplifyFunctionsPlus : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

public:
	/**
	 * @brief Simplify mesh to target percentage, copy UV and Normal to overlay
	 * @param TargetMesh 
	 * @param Options 
	 * @param TargetPercentage 
	 * @param Debug 
	 * @return 
	 */
	UFUNCTION(BlueprintCallable, Category = "GeometryScript|Simplification", meta=(ScriptMethod))
	static UPARAM(DisplayName = "Target Mesh") UDynamicMesh*
	ApplySimplifyToTargetPercentage(
		UDynamicMesh* TargetMesh,
		FGeometryScriptPlusSimplifyMeshOptions Options,
		int32 TargetPercentage = 50,
		UGeometryScriptDebug* Debug = nullptr);

private:
	static UDynamicMesh* CalculateResult(UDynamicMesh* TargetMesh, FGeometryScriptPlusSimplifyMeshOptions Options,
	                              const ERuntimeSimplifyTargetType TargetMode,
	                              const float TargetPercentage = 50, const int TargetCount = 1000, const float TargetEdgeLength = 5,
	                              const float PolyEdgeAngleTolerance = 0.1);
};
