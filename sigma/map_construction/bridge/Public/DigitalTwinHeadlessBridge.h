// Copyright (c) 2026
#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "DigitalTwinHeadlessBridge.generated.h"

class UUserWidget;

/**
 * Reflection bridge for invoking the real compiled Blueprint button event
 * of CARLA's official Digital Twin Tool through an unattended editor pass.
 */
UCLASS()
class CARLATOOLS_API UDigitalTwinHeadlessBridge
  : public UBlueprintFunctionLibrary
{
  GENERATED_BODY()

public:
  /** List all zero-parameter button/click Blueprint functions in root and child widgets. */
  UFUNCTION(BlueprintCallable, Category="CARLA|DigitalTwin|Headless")
  static TArray<FString> ListButtonEventsRecursive(UUserWidget* RootWidget);

  /**
   * Find the best zero-parameter button event containing NameToken.
   * The returned entry is "<object path>||<UFunction name>".
   */
  UFUNCTION(BlueprintCallable, Category="CARLA|DigitalTwin|Headless")
  static FString FindButtonEventRecursive(
      UUserWidget* RootWidget,
      const FString& NameToken);

  /** Invoke an entry returned by FindButtonEventRecursive using UObject::ProcessEvent. */
  UFUNCTION(BlueprintCallable, Category="CARLA|DigitalTwin|Headless")
  static bool InvokeButtonEventRecursive(
      UUserWidget* RootWidget,
      const FString& Entry);

  /**
   * Set text/value on all descendant widgets whose object name contains NameToken.
   * Supports EditableTextBox, EditableText, MultiLineEditableTextBox,
   * MultiLineEditableText and SpinBox.
   */
  UFUNCTION(BlueprintCallable, Category="CARLA|DigitalTwin|Headless")
  static int32 SetWidgetValueByNameRecursive(
      UUserWidget* RootWidget,
      const FString& NameToken,
      const FString& Value);

  /** Return live UButton widgets as "<object path>||<visible text>". */
  UFUNCTION(BlueprintCallable, Category="CARLA|DigitalTwin|Headless")
  static TArray<FString> ListButtonsByVisibleTextRecursive(
      UUserWidget* RootWidget);

  /** Find a unique button by visible text; returns its object path. */
  UFUNCTION(BlueprintCallable, Category="CARLA|DigitalTwin|Headless")
  static FString FindButtonByVisibleTextRecursive(
      UUserWidget* RootWidget,
      const FString& VisibleText);

  /** Broadcast the real UButton::OnClicked delegate selected by visible text. */
  UFUNCTION(BlueprintCallable, Category="CARLA|DigitalTwin|Headless")
  static bool ClickButtonByVisibleTextRecursive(
      UUserWidget* RootWidget,
      const FString& VisibleText);

};
