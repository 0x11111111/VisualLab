#include"stdafx.h"

class DesignInteractorStyle : public vtkInteractorStyleTrackballCamera
{
public:
    static DesignInteractorStyle* New()
    {
        return new DesignInteractorStyle;
    }
    vtkTypeMacro(DesignInteractorStyle, vtkInteractorStyleTrackballCamera);

    DesignInteractorStyle() {}
    virtual ~DesignInteractorStyle() {}
    virtual void OnLeftButtonDown() {}
    virtual void OnLeftButtonUp() {}
    virtual void OnRightButtonDown() { this->StartRotate(); }
    virtual void OnRightButtonUp() { this->vtkInteractorStyleTrackballCamera::OnLeftButtonUp(); }
    virtual void OnMouseMove() { this->vtkInteractorStyleTrackballCamera::OnMouseMove(); }
    virtual void OnMouseWheelForward() { if (!this->Interactor->GetControlKey() && !this->Interactor->GetShiftKey()) this->vtkInteractorStyleTrackballCamera::OnMouseWheelForward(); }
    virtual void OnMouseWheelBackward() { if (!this->Interactor->GetControlKey() && !this->Interactor->GetShiftKey()) this->vtkInteractorStyleTrackballCamera::OnMouseWheelBackward(); }
};

class vtkRenderPipeline
{
public:
    vtkRenderPipeline()
    {
		this->RenderWindow = vtkSmartPointer<vtkRenderWindow>::New();
		this->Renderer = vtkSmartPointer<vtkRenderer>::New();
		this->CellPicker = vtkSmartPointer<vtkCellPicker>::New();
		this->Renderer->SetBackground(0.41, 0.41, 0.41);
		this->RenderWindow->AddRenderer(this->Renderer);
		this->RenderWindow->SetSize(800, 800);
		this->RenderWindow->Render();
		this->InteractorStyle = vtkSmartPointer<DesignInteractorStyle>::New();
		this->RenderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
		this->RenderWindowInteractor->SetInteractorStyle(this->InteractorStyle);
		this->RenderWindowInteractor->SetRenderWindow(this->RenderWindow);
	}
    void addObserver(unsigned long event,void (*f)(vtkObject* caller, unsigned long eid,void* clientdata, void* calldata))
    {
		vtkNew<vtkCallbackCommand> callback;
		callback->SetCallback(f);
		callback->InitializeObjectBase();
		this->RenderWindowInteractor->AddObserver(event, callback);
	}

    vtkSmartPointer<vtkRenderWindow> RenderWindow;
    vtkSmartPointer<vtkRenderer> Renderer;
    vtkSmartPointer<vtkCellPicker> CellPicker;
    vtkSmartPointer<DesignInteractorStyle> InteractorStyle;
    vtkSmartPointer<vtkRenderWindowInteractor> RenderWindowInteractor;
};