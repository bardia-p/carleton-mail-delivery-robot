const submitButton = $("#registerSuperuser-submit");
const errorMessage = $("#error-message");

/**
 * The JavaScript AJAX call for when a new superuser is registered.
 */
submitButton.click((e) => {
    e.preventDefault();
    const dataDictionary = {};
    if (!$("#username").val()) {
        setToast("warning", "No Input", "Please input a username");
        return;
    }
    else {
        dataDictionary["username"] = $("#username").val();
    }
    if (!$("#password").val()) {
        setToast("warning", "No Input", "Please input a password");
        return;
    }
    else {
        dataDictionary["password"] = $("#password").val();
    }
    var formData = JSON.stringify(dataDictionary);

    console.log(formData);
    console.log("Got here");

    $.ajax({
        type: "POST",
        url: "/api/v1/createSuperuser",
        data: formData,
        dataType: "json",
        contentType: "application/json",
        success: function (res) {
            console.log('Superuser registered successfully');
            if (res === 200) {
                setToast("success", "", "Successfully registered superuser", true);
                window.location.href = "/";
            }
            else if (res === 401) errorMessage.text("This username already exists!")
        },
        error: function (xhr, status, error) {
            // error handling
            console.error('Error creating user:', error);
            setToast("error", "Something went wrong", "Could not create user");
        }
    })

});