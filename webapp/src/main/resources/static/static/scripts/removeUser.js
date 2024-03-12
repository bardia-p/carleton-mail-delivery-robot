const submitButton = $("#removeUser-submit");
const errorMessage = $("#error-message");

/**
 * The JavaScript AJAX call for when an existing user is removed.
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
    var formData = JSON.stringify(dataDictionary);

    console.log(formData);
    console.log("Got here");

    $.ajax({
        type: "POST",
        url: "/api/v1/removeUser",
        data: formData,
        dataType: "json",
        contentType: "application/json",
        success: function (res) {
            console.log('User deregistered successfully');
            if (res === 200) {
                setToast("success", "", "Successfully deregistered user", true);
                window.location.href = "/admin";
            }
            else if (res === 401) errorMessage.text("This username doesn't exist!")
        },
        error: function (xhr, status, error) {
            // error handling
            console.error('Error removing user:', error);
            setToast("error", "Something went wrong", "Could not remove user");
        }
    })

});