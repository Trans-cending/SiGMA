// Copyright (c) 2026

#include "DigitalTwinHeadlessBridge.h"

#include "Blueprint/UserWidget.h"
#include "Blueprint/WidgetTree.h"
#include "Components/Button.h"
#include "Components/ContentWidget.h"
#include "Components/EditableText.h"
#include "Components/EditableTextBox.h"
#include "Components/MultiLineEditableText.h"
#include "Components/MultiLineEditableTextBox.h"
#include "Components/PanelWidget.h"
#include "Components/SpinBox.h"
#include "Components/TextBlock.h"
#include "Components/Widget.h"
#include "UObject/Class.h"
#include "UObject/FieldIterator.h"

namespace
{

static const TCHAR* EntrySeparator = TEXT("||");

void CollectWidgetObjects(
    UUserWidget* RootWidget,
    TArray<UObject*>& OutObjects,
    TSet<UObject*>& Visited)
{
  if (RootWidget == nullptr || Visited.Contains(RootWidget))
  {
    return;
  }

  Visited.Add(RootWidget);
  OutObjects.Add(RootWidget);

  if (RootWidget->WidgetTree == nullptr)
  {
    return;
  }

  TArray<UWidget*> Widgets;
  RootWidget->WidgetTree->GetAllWidgets(Widgets);

  for (UWidget* Widget : Widgets)
  {
    if (Widget == nullptr || Visited.Contains(Widget))
    {
      continue;
    }

    Visited.Add(Widget);
    OutObjects.Add(Widget);

    if (UUserWidget* ChildUserWidget = Cast<UUserWidget>(Widget))
    {
      CollectWidgetObjects(ChildUserWidget, OutObjects, Visited);
    }
  }
}

TArray<UObject*> CollectWidgetObjects(UUserWidget* RootWidget)
{
  TArray<UObject*> Objects;
  TSet<UObject*> Visited;
  CollectWidgetObjects(RootWidget, Objects, Visited);
  return Objects;
}

bool IsZeroParameterFunction(const UFunction* Function)
{
  return Function != nullptr && Function->ParmsSize == 0;
}

bool LooksLikeButtonEvent(const FString& FunctionName)
{
  return
      FunctionName.Contains(TEXT("OnButtonClickedEvent"), ESearchCase::IgnoreCase) ||
      FunctionName.Contains(TEXT("OnClicked"), ESearchCase::IgnoreCase) ||
      (
        FunctionName.Contains(TEXT("Button"), ESearchCase::IgnoreCase) &&
        FunctionName.Contains(TEXT("Event"), ESearchCase::IgnoreCase)
      );
}

int32 CandidateScore(const FString& FunctionName, const FString& NameToken)
{
  if (!FunctionName.Contains(NameToken, ESearchCase::IgnoreCase))
  {
    return -1;
  }

  int32 Score = 0;

  if (FunctionName.StartsWith(TEXT("BndEvt__"), ESearchCase::IgnoreCase))
  {
    Score += 1000;
  }
  if (FunctionName.Contains(TEXT("OnButtonClickedEvent"), ESearchCase::IgnoreCase))
  {
    Score += 500;
  }
  if (FunctionName.Contains(TEXT("K2Node_ComponentBoundEvent"), ESearchCase::IgnoreCase))
  {
    Score += 250;
  }
  if (FunctionName.Contains(TEXT("GenerateButton"), ESearchCase::IgnoreCase) ||
      FunctionName.Contains(TEXT("GenerateMapButton"), ESearchCase::IgnoreCase))
  {
    Score += 200;
  }
  if (FunctionName.Contains(TEXT("Generate"), ESearchCase::IgnoreCase))
  {
    Score += 100;
  }

  return Score;
}

FString MakeEntry(UObject* Target, UFunction* Function)
{
  return Target->GetPathName() + EntrySeparator + Function->GetName();
}

bool SplitEntry(const FString& Entry, FString& ObjectPath, FString& FunctionName)
{
  return Entry.Split(
      EntrySeparator,
      &ObjectPath,
      &FunctionName,
      ESearchCase::CaseSensitive,
      ESearchDir::FromStart);
}


void AppendVisibleText(UWidget* Widget, TArray<FString>& Parts)
{
  if (Widget == nullptr)
  {
    return;
  }

  if (UTextBlock* TextBlock = Cast<UTextBlock>(Widget))
  {
    const FString Value = TextBlock->GetText().ToString().TrimStartAndEnd();
    if (!Value.IsEmpty())
    {
      Parts.Add(Value);
    }
    return;
  }

  if (UPanelWidget* Panel = Cast<UPanelWidget>(Widget))
  {
    for (int32 Index = 0; Index < Panel->GetChildrenCount(); ++Index)
    {
      AppendVisibleText(Panel->GetChildAt(Index), Parts);
    }
    return;
  }

  if (UContentWidget* Content = Cast<UContentWidget>(Widget))
  {
    AppendVisibleText(Content->GetContent(), Parts);
  }
}

FString GetButtonVisibleText(UButton* Button)
{
  if (Button == nullptr)
  {
    return FString();
  }

  TArray<FString> Parts;
  AppendVisibleText(Button->GetContent(), Parts);

  FString Result;
  for (const FString& Part : Parts)
  {
    if (!Result.IsEmpty())
    {
      Result += TEXT(" ");
    }
    Result += Part;
  }
  return Result.TrimStartAndEnd();
}

TArray<UButton*> CollectButtons(UUserWidget* RootWidget)
{
  TArray<UButton*> Buttons;
  for (UObject* Object : CollectWidgetObjects(RootWidget))
  {
    if (UButton* Button = Cast<UButton>(Object))
    {
      Buttons.AddUnique(Button);
    }
  }
  return Buttons;
}

UButton* FindButtonByVisibleText(
    UUserWidget* RootWidget,
    const FString& RequestedText)
{
  const FString Needle = RequestedText.TrimStartAndEnd();
  TArray<UButton*> ExactMatches;
  TArray<UButton*> ContainsMatches;

  for (UButton* Button : CollectButtons(RootWidget))
  {
    const FString Label = GetButtonVisibleText(Button);
    if (Label.Equals(Needle, ESearchCase::IgnoreCase))
    {
      ExactMatches.Add(Button);
    }
    else if (!Needle.IsEmpty() &&
             Label.Contains(Needle, ESearchCase::IgnoreCase))
    {
      ContainsMatches.Add(Button);
    }
  }

  if (ExactMatches.Num() == 1)
  {
    return ExactMatches[0];
  }
  if (ExactMatches.Num() == 0 && ContainsMatches.Num() == 1)
  {
    return ContainsMatches[0];
  }

  UE_LOG(
      LogTemp,
      Error,
      TEXT("DTT button label '%s': exact=%d contains=%d"),
      *RequestedText,
      ExactMatches.Num(),
      ContainsMatches.Num());
  return nullptr;
}

} // namespace

TArray<FString> UDigitalTwinHeadlessBridge::ListButtonEventsRecursive(
    UUserWidget* RootWidget)
{
  TArray<FString> Results;

  for (UObject* Target : CollectWidgetObjects(RootWidget))
  {
    if (Target == nullptr || Target->GetClass() == nullptr)
    {
      continue;
    }

    for (TFieldIterator<UFunction> It(
             Target->GetClass(),
             EFieldIteratorFlags::IncludeSuper);
         It;
         ++It)
    {
      UFunction* Function = *It;
      if (!IsZeroParameterFunction(Function))
      {
        continue;
      }

      const FString FunctionName = Function->GetName();
      if (LooksLikeButtonEvent(FunctionName))
      {
        Results.AddUnique(MakeEntry(Target, Function));
      }
    }
  }

  Results.Sort();
  return Results;
}

FString UDigitalTwinHeadlessBridge::FindButtonEventRecursive(
    UUserWidget* RootWidget,
    const FString& NameToken)
{
  FString BestEntry;
  int32 BestScore = -1;

  for (UObject* Target : CollectWidgetObjects(RootWidget))
  {
    if (Target == nullptr || Target->GetClass() == nullptr)
    {
      continue;
    }

    for (TFieldIterator<UFunction> It(
             Target->GetClass(),
             EFieldIteratorFlags::IncludeSuper);
         It;
         ++It)
    {
      UFunction* Function = *It;
      if (!IsZeroParameterFunction(Function))
      {
        continue;
      }

      const FString FunctionName = Function->GetName();
      if (!LooksLikeButtonEvent(FunctionName))
      {
        continue;
      }

      const int32 Score = CandidateScore(FunctionName, NameToken);
      if (Score > BestScore)
      {
        BestScore = Score;
        BestEntry = MakeEntry(Target, Function);
      }
    }
  }

  UE_LOG(
      LogTemp,
      Display,
      TEXT("DigitalTwinHeadlessBridge Find token='%s' entry='%s' score=%d"),
      *NameToken,
      *BestEntry,
      BestScore);

  return BestEntry;
}

bool UDigitalTwinHeadlessBridge::InvokeButtonEventRecursive(
    UUserWidget* RootWidget,
    const FString& Entry)
{
  FString ObjectPath;
  FString FunctionName;

  if (!SplitEntry(Entry, ObjectPath, FunctionName))
  {
    UE_LOG(LogTemp, Error, TEXT("Invalid DTT entry: %s"), *Entry);
    return false;
  }

  for (UObject* Target : CollectWidgetObjects(RootWidget))
  {
    if (Target == nullptr || Target->GetPathName() != ObjectPath)
    {
      continue;
    }

    UFunction* Function = Target->FindFunction(FName(*FunctionName));
    if (!IsZeroParameterFunction(Function))
    {
      UE_LOG(
          LogTemp,
          Error,
          TEXT("DTT function missing or has parameters: %s"),
          *Entry);
      return false;
    }

    UE_LOG(
        LogTemp,
        Display,
        TEXT("MOSAIC_DTT_PROCESS_EVENT_BEGIN=%s"),
        *Entry);

    Target->ProcessEvent(Function, nullptr);

    UE_LOG(
        LogTemp,
        Display,
        TEXT("MOSAIC_DTT_PROCESS_EVENT_RETURNED=%s"),
        *Entry);

    return true;
  }

  UE_LOG(LogTemp, Error, TEXT("DTT event owner not found: %s"), *Entry);
  return false;
}

int32 UDigitalTwinHeadlessBridge::SetWidgetValueByNameRecursive(
    UUserWidget* RootWidget,
    const FString& NameToken,
    const FString& Value)
{
  int32 Count = 0;
  const FText TextValue = FText::FromString(Value);
  const float NumberValue = FCString::Atof(*Value);
  static const char MosaicWidgetCommitBuildMarker[] =
      "MOSAIC_DTT_WIDGET_COMMIT_BROADCAST";

  for (UObject* Object : CollectWidgetObjects(RootWidget))
  {
    UWidget* Widget = Cast<UWidget>(Object);
    if (Widget == nullptr)
    {
      continue;
    }

    const FString SearchName =
        Widget->GetName() + TEXT(" ") + Widget->GetPathName();

    if (!SearchName.Contains(NameToken, ESearchCase::IgnoreCase))
    {
      continue;
    }

    if (UEditableTextBox* EditableTextBox = Cast<UEditableTextBox>(Widget))
    {
      EditableTextBox->SetText(TextValue);
      EditableTextBox->OnTextChanged.Broadcast(TextValue);
      EditableTextBox->OnTextCommitted.Broadcast(
          TextValue, ETextCommit::OnEnter);
      ++Count;
    }
    else if (UEditableText* EditableText = Cast<UEditableText>(Widget))
    {
      EditableText->SetText(TextValue);
      EditableText->OnTextChanged.Broadcast(TextValue);
      EditableText->OnTextCommitted.Broadcast(
          TextValue, ETextCommit::OnEnter);
      ++Count;
    }
    else if (UMultiLineEditableTextBox* MultiLineBox =
                 Cast<UMultiLineEditableTextBox>(Widget))
    {
      MultiLineBox->SetText(TextValue);
      MultiLineBox->OnTextChanged.Broadcast(TextValue);
      MultiLineBox->OnTextCommitted.Broadcast(
          TextValue, ETextCommit::OnEnter);
      ++Count;
    }
    else if (UMultiLineEditableText* MultiLine =
                 Cast<UMultiLineEditableText>(Widget))
    {
      MultiLine->SetText(TextValue);
      MultiLine->OnTextChanged.Broadcast(TextValue);
      MultiLine->OnTextCommitted.Broadcast(
          TextValue, ETextCommit::OnEnter);
      ++Count;
    }
    else if (USpinBox* SpinBox = Cast<USpinBox>(Widget))
    {
      SpinBox->SetValue(NumberValue);
      SpinBox->OnValueChanged.Broadcast(NumberValue);
      SpinBox->OnValueCommitted.Broadcast(
          NumberValue, ETextCommit::OnEnter);
      ++Count;
    }
  }

  UE_LOG(
      LogTemp,
      Display,
      TEXT("%s set token='%s' value='%s' count=%d"),
      UTF8_TO_TCHAR(MosaicWidgetCommitBuildMarker),
      *NameToken,
      *Value,
      Count);

  return Count;
}


TArray<FString>
UDigitalTwinHeadlessBridge::ListButtonsByVisibleTextRecursive(
    UUserWidget* RootWidget)
{
  TArray<FString> Results;
  for (UButton* Button : CollectButtons(RootWidget))
  {
    Results.Add(
        Button->GetPathName() + EntrySeparator + GetButtonVisibleText(Button));
  }
  Results.Sort();
  return Results;
}

FString UDigitalTwinHeadlessBridge::FindButtonByVisibleTextRecursive(
    UUserWidget* RootWidget,
    const FString& VisibleText)
{
  UButton* Button = FindButtonByVisibleText(RootWidget, VisibleText);
  return Button == nullptr ? FString() : Button->GetPathName();
}

bool UDigitalTwinHeadlessBridge::ClickButtonByVisibleTextRecursive(
    UUserWidget* RootWidget,
    const FString& VisibleText)
{
  UButton* Button = FindButtonByVisibleText(RootWidget, VisibleText);
  if (Button == nullptr)
  {
    return false;
  }

  UE_LOG(
      LogTemp,
      Display,
      TEXT("MOSAIC_DTT_BUTTON_CLICK_BEGIN=%s||%s"),
      *Button->GetPathName(),
      *GetButtonVisibleText(Button));

  Button->OnClicked.Broadcast();

  UE_LOG(
      LogTemp,
      Display,
      TEXT("MOSAIC_DTT_BUTTON_CLICK_RETURNED=%s||%s"),
      *Button->GetPathName(),
      *GetButtonVisibleText(Button));
  return true;
}
